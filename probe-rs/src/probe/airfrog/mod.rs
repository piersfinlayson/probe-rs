//! airfrog driver - see https://piers.rocks/u/airfrog
//!
//! airfrog supports SWD over WiFi using a proprietary binary protocol.  This
//! driver implements client side of that protocol:
//! https://github.com/piersfinlayson/airfrog/blob/main/docs/REST-API.md

use crate::architecture::arm::{
    ArmCommunicationInterface, ArmDebugInterface, sequences::ArmDebugSequence,
};
use crate::{
    CoreStatus,
    architecture::arm::{
        ArmError, RawDapAccess, RegisterAddress, communication_interface::DapProbe,
    },
    probe::{
        BoxedProbeError, DebugProbe, DebugProbeError, DebugProbeInfo, DebugProbeSelector,
        ProbeCreationError, ProbeError, ProbeFactory, WireProtocol,
    },
};
use airfrog_bin::{
    CMD_AP_BULK_READ, CMD_AP_BULK_WRITE, CMD_AP_READ, CMD_AP_WRITE, CMD_DISCONNECT, CMD_DP_READ,
    CMD_DP_WRITE, CMD_MULTI_REG_WRITE, CMD_RESET_TARGET, CMD_SET_SPEED, MAX_WORD_COUNT, PORT,
    RSP_OK, RegType, Speed, VERSION,
};
use std::io::{Read, Write};
use std::sync::Arc;

const AIRFROG_STR: &str = "Airfrog";

/// Airfrog probe factory
#[derive(Debug)]
pub struct AirfrogFactory;

impl std::fmt::Display for AirfrogFactory {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{AIRFROG_STR}")
    }
}

// Struct to hold cached up single register writes, in order to batch them up
// before sending
#[derive(Debug)]
struct WriteReg {
    /// Address to write to
    address: RegisterAddress,

    /// Data to write
    data: u32,
}

// Struct to hold AP write blocks, in order to batch them up before sending
#[derive(Debug)]
struct WriteBlock {
    /// Address to write to
    address: RegisterAddress,

    /// Data to write
    data: Vec<u32>,
}

/// Airfrog probe implementation
#[derive(Debug)]
pub struct AirfrogProbe {
    /// Host address
    host: String,

    /// Port number
    port: u16,

    /// Current SWD speed in kHz
    speed: Speed,

    /// TCP stream for binary protocol
    stream: Option<std::net::TcpStream>,

    // Queued up single write operations to be sent
    write_regs: Vec<WriteReg>,

    // Queued up AP write blocks to be sent
    ap_write_blocks: Vec<WriteBlock>,
}

/// Airfrog-specific errors
#[derive(thiserror::Error, Debug)]
pub enum AirfrogError {
    /// IO error  
    #[error("Airfrog probe communication failed: {0}")]
    Io(#[from] std::io::Error),

    /// Invalid response from airfrog
    #[error("Airfrog probe returned invalid response: {0}")]
    InvalidResponse(String),

    /// Airfrog command failed
    #[error("Airfrog probe command failed: {0}")]
    CommandFailed(String),

    /// Not attached to an Airfrog probe
    #[error("Not attached to Airfrog probe")]
    NotAttached,

    /// Invalid URL
    #[error("Invalid Airfrog IP/port: {0}")]
    InvalidUrl(String),

    /// Internal Error
    #[error("Airfrog driver internal error - please raise an issue: {0}")]
    InternalError(String),
}

impl ProbeError for AirfrogError {}

impl AirfrogProbe {
    /// Create a new airfrog probe
    pub fn new(address: String) -> Result<Self, AirfrogError> {
        let (host, port) = if let Some((h, p)) = address.split_once(':') {
            let port = p
                .parse::<u16>()
                .map_err(|_| AirfrogError::InvalidUrl(format!("Port: {p}")))?;
            (h.to_string(), port)
        } else {
            // Default port if not specified
            (address, PORT)
        };

        Ok(Self {
            host,
            port,
            speed: Speed::default(),
            stream: None,
            write_regs: Vec::new(),
            ap_write_blocks: Vec::new(),
        })
    }
}

// Binary protocol helpers
impl AirfrogProbe {
    // Single function that sends a command, waits for the response, and
    // checks it was successful.  It handles errors in a common way, allowing
    // the Airfrog probe-rs trait implementations to be as simple as possible.
    //
    // Arguments:
    // - send_data: Data to send - a command and, for writes, the data to write
    // - expected_len: Expected length of the response, in addition to the
    //   first, status, byte.
    //
    // Returns:
    // - Ok(None) if the command was successful and no data is expected back
    // - Ok(Some(data)) if the command was successful and data is expected
    // - Err(DebugProbeError) if there was an error in sending the command,
    //
    // If a non zero value was passed in `expected_len`, a success response is
    // guaranteed to be Ok(Some(data)), not None, to allow the caller to
    // unwrap safely.
    fn send_recv_read_airfrog(
        &mut self,
        send_data: &[u8],
        expected_len: usize,
    ) -> Result<Option<Vec<u8>>, DebugProbeError> {
        // Send th command
        self.send_command(send_data)?;

        // Receive the response
        let result = self.read_response(expected_len + 1);

        // Process all possible error cases
        let response = if let Err(e) = result {
            return Err(DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(e))));
        } else {
            result.unwrap()
        };
        if response.is_empty() {
            return Err(DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(
                AirfrogError::InvalidResponse("Empty response from probe".to_string()),
            ))));
        } else if response[0] != RSP_OK {
            return Err(DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(
                AirfrogError::CommandFailed(format!(
                    "Probe command failed with status: {:#04X}",
                    response[0]
                )),
            ))));
        } else if response.len() != expected_len + 1 {
            return Err(DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(
                AirfrogError::InvalidResponse(format!(
                    "Unexpected response length from probe: expected {}, got {}",
                    expected_len + 1,
                    response.len()
                )),
            ))));
        }

        // Success!
        if response.len() == 1 {
            if expected_len > 0 {
                // This can only happen if our above logic was flawed, but we
                // explicitly check it as we guarantee that if `expected_len`
                // is non-zero, we will return Some(data).
                Err(DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(
                    AirfrogError::InternalError(
                        "Expected non-zero response length, but got 0".to_string(),
                    ),
                ))))
            } else {
                Ok(None)
            }
        } else {
            Ok(Some(response[1..].to_vec()))
        }
    }

    // Single function to send a command and expect only the status byte back.
    //
    // A simplified version of `send_recv_read_airfrog()`.
    fn send_recv_airfrog(&mut self, send_data: &[u8]) -> Result<(), DebugProbeError> {
        self.send_recv_read_airfrog(send_data, 0).map(|_| ())
    }

    // Sends a binary API command command over TCP to the Airfrog probe.
    fn send_command(&mut self, data: &[u8]) -> Result<(), AirfrogError> {
        let stream = self.stream.as_mut().ok_or(AirfrogError::NotAttached)?;
        stream.write_all(data)?;
        Ok(())
    }

    // Reads a response from the Airfrog probe.  Blocks until the expected
    // number of bytes have been read.
    fn read_response(&mut self, len: usize) -> Result<Vec<u8>, AirfrogError> {
        let stream = self.stream.as_mut().ok_or(AirfrogError::NotAttached)?;
        let mut buf = vec![0u8; len];
        stream.read_exact(&mut buf)?;
        Ok(buf)
    }

    // Sets the speed of the Airfrog probe over the binary API.
    fn set_speed_airfrog(&mut self, speed: Speed) -> Result<(), DebugProbeError> {
        // Get airfrog speed based on Speed enum
        let command = [CMD_SET_SPEED, speed as u8];
        self.send_recv_airfrog(&command)?;

        // Update our internal view of speed
        self.speed = speed;
        Ok(())
    }

    // Write a block of data to a single AP register.  This sends makes a
    // single binary API call to send all of the data, up to
    // MAX_WORD_COUNT
    fn internal_write_ap_block(&mut self, block: WriteBlock) -> Result<(), ArmError> {
        let address = block.address;
        let values = block.data;

        match address {
            RegisterAddress::DpRegister(_) => {
                unreachable!("DP blocks not supported by internal_write_ap_block")
            }
            RegisterAddress::ApRegister(_) => {
                let count = values.len();
                if count > MAX_WORD_COUNT as usize {
                    return Err(ArmError::Probe(DebugProbeError::Other(format!(
                        "Airfrog API bulk write limit exceeded - {MAX_WORD_COUNT} words vs {count}"
                    ))));
                }

                let reg = address.lsb();
                let count = values.len() as u16;

                // Send: [cmd][reg][count:2][data...]
                let mut command = vec![CMD_AP_BULK_WRITE, reg];
                command.extend_from_slice(&count.to_le_bytes());
                for val in values {
                    command.extend_from_slice(&val.to_le_bytes());
                }

                self.send_recv_airfrog(&command)?;
                Ok(())
            }
        }
    }

    // Called by raw_flush() to flush any queued up AP write blocks.
    fn flush_write_blocks(&mut self) -> Result<(), ArmError> {
        // Get all the blocks to write
        let write_blocks: Vec<_> = self.ap_write_blocks.drain(..).collect();

        if write_blocks.is_empty() {
            return Ok(());
        }

        // See if they are all for the same register address - if so, we can
        // combine them into a single bulk write.  Even if a few sequential
        // ones were the same and the others weren't we could, but we won't
        // bother as it's unlikely.
        let first_addr = write_blocks[0].address;
        let all_same_addr = write_blocks.iter().all(|block| block.address == first_addr);

        if all_same_addr {
            // Create a single write
            let combined_data: Vec<u32> = write_blocks
                .into_iter()
                .flat_map(|block| block.data)
                .collect();

            // Send in chunks of max size
            for chunk in combined_data.chunks(MAX_WORD_COUNT as usize) {
                let chunk_block = WriteBlock {
                    address: first_addr,
                    data: chunk.to_vec(),
                };
                self.internal_write_ap_block(chunk_block)?;
            }
        } else {
            // Fall back to individual writes of blocks
            for block in write_blocks {
                self.internal_write_ap_block(block)?;
            }
        }

        Ok(())
    }

    fn flush_write_reg(&mut self) -> Result<(), ArmError> {
        // Get all the single writes to do
        let write_regs: Vec<_> = self.write_regs.drain(..).collect();

        if write_regs.is_empty() {
            return Ok(());
        } else if write_regs.len() == 1 {
            // If there's only one, we can just write it directly
            let (cmd, reg) = match write_regs[0].address {
                RegisterAddress::DpRegister(_) => (CMD_DP_WRITE, write_regs[0].address.lsb()),
                RegisterAddress::ApRegister(_) => (CMD_AP_WRITE, write_regs[0].address.lsb()),
            };
            let mut command = vec![cmd, reg];
            command.extend_from_slice(&write_regs[0].data.to_le_bytes());
            self.send_recv_airfrog(&command)?;
            return Ok(());
        }

        // More than one write - build the MultiRegWrite command
        let mut command = Vec::with_capacity(1 + 2 + (6 * write_regs.len()));
        command.push(CMD_MULTI_REG_WRITE);
        command.extend_from_slice(&(write_regs.len() as u16).to_le_bytes());

        for reg in &write_regs {
            // Add the command byte
            let cmd = match reg.address {
                RegisterAddress::DpRegister(_) => RegType::Dp as u8,
                RegisterAddress::ApRegister(_) => RegType::Ap as u8,
            };
            command.push(cmd);

            // Add the register address
            command.push(reg.address.lsb());

            // Add the data
            command.extend_from_slice(&reg.data.to_le_bytes());
        }

        self.send_recv_airfrog(&command)?;

        Ok(())
    }
}

impl ProbeFactory for AirfrogFactory {
    fn list_probes(&self) -> Vec<DebugProbeInfo> {
        // No detection for airfrog probes
        vec![]
    }

    // Accept any probe whose serial number starts with "airfrog:",
    // irrespective of the VID:PID.
    fn list_probes_filtered(&self, selector: Option<&DebugProbeSelector>) -> Vec<DebugProbeInfo> {
        let Some(selector) = selector else {
            return self.list_probes();
        };

        let Some(serial) = selector.serial_number.as_deref() else {
            return vec![];
        };

        // Check if it starts with "airfrog:"
        if let Some(address) = serial.strip_prefix("airfrog:") {
            vec![DebugProbeInfo::new(
                format!("Airfrog ({address})"),
                selector.vendor_id,
                selector.product_id,
                Some(serial.to_string()),
                &AirfrogFactory,
                None,
            )]
        } else {
            vec![]
        }
    }

    fn open(&self, selector: &DebugProbeSelector) -> Result<Box<dyn DebugProbe>, DebugProbeError> {
        // Check VID:PID is 0000:0000
        if (selector.vendor_id, selector.product_id) != (0, 0) {
            return Err(DebugProbeError::ProbeCouldNotBeCreated(
                ProbeCreationError::NotFound,
            ));
        }

        // Check serial number starts with "airfrog:"
        let serial = selector
            .serial_number
            .as_ref()
            .ok_or_else(|| DebugProbeError::ProbeCouldNotBeCreated(ProbeCreationError::NotFound))?;
        let address = serial
            .strip_prefix("airfrog:")
            .ok_or_else(|| DebugProbeError::ProbeCouldNotBeCreated(ProbeCreationError::NotFound))?;

        // Create the Airfrog probe instance based on the address/port string
        // remaining in the serial number
        let probe = AirfrogProbe::new(address.to_string())
            .map_err(|e| DebugProbeError::ProbeCouldNotBeCreated(ProbeCreationError::from(e)))?;

        Ok(Box::new(probe))
    }
}

impl DebugProbe for AirfrogProbe {
    fn get_name(&self) -> &str {
        AIRFROG_STR
    }

    fn speed_khz(&self) -> u32 {
        self.speed.to_khz()
    }

    fn set_speed(&mut self, speed_khz: u32) -> Result<u32, DebugProbeError> {
        let speed = Speed::from_khz(speed_khz);
        self.set_speed_airfrog(speed).map(|_| self.speed.to_khz())
    }

    fn attach(&mut self) -> Result<(), DebugProbeError> {
        // Create a TCP connection to the Airfrog probe
        let addr = format!("{}:{}", self.host, self.port);
        let mut stream = std::net::TcpStream::connect(&addr).map_err(|e| {
            DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(AirfrogError::Io(e))))
        })?;

        // We don't want to buffer small packets - most of our packets will be
        // small
        stream.set_nodelay(true).map_err(|e| {
            DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(AirfrogError::Io(e))))
        })?;

        // Set timeouts to avoid hanging indefinitely in the case of network
        // isues
        stream
            .set_read_timeout(Some(std::time::Duration::from_secs(1)))
            .map_err(|e| {
                DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(AirfrogError::Io(e))))
            })?;
        stream
            .set_write_timeout(Some(std::time::Duration::from_secs(1)))
            .map_err(|e| {
                DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(AirfrogError::Io(e))))
            })?;

        // Binary API handshake - first read version from server
        let mut version_buf = [0u8; 1];
        stream.read_exact(&mut version_buf).map_err(|e| {
            DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(AirfrogError::Io(e))))
        })?;

        // Then send version
        stream.write_all(&[VERSION]).map_err(|e| {
            DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(AirfrogError::Io(e))))
        })?;

        // All good so far.  Set self.stream so we can use our internal
        // methods.  However, if we fail in this function, we have to set this
        // back to None.
        self.stream = Some(stream);

        // Set the speed, to the default, in case the airfrog was otherwise
        // configured
        self.set_speed_airfrog(self.speed)
            .inspect_err(|_| self.stream = None)
    }

    // Doesn't seem to be called by probe-rs when using Ctrl-C to exit probe-rs.
    fn detach(&mut self) -> Result<(), crate::Error> {
        if let Some(mut stream) = self.stream.take() {
            // Send binary API disconnect and flush it
            stream.write_all(&[CMD_DISCONNECT]).map_err(|e| {
                DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(AirfrogError::Io(e))))
            })?;
            stream.flush().map_err(|e| {
                DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(AirfrogError::Io(e))))
            })?;

            // Close the TCP connection
            stream.shutdown(std::net::Shutdown::Both).map_err(|e| {
                DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(AirfrogError::Io(e))))
            })?;
        }

        // Done - clean up
        self.stream = None;
        Ok(())
    }

    fn target_reset(&mut self) -> Result<(), DebugProbeError> {
        self.send_recv_airfrog(&[CMD_RESET_TARGET])
    }

    fn target_reset_assert(&mut self) -> Result<(), DebugProbeError> {
        Err(DebugProbeError::NotImplemented {
            function_name: "target_reset_assert",
        })
    }

    fn target_reset_deassert(&mut self) -> Result<(), DebugProbeError> {
        Err(DebugProbeError::NotImplemented {
            function_name: "target_reset_deassert",
        })
    }

    fn select_protocol(&mut self, protocol: WireProtocol) -> Result<(), DebugProbeError> {
        match protocol {
            WireProtocol::Swd => Ok(()),
            WireProtocol::Jtag => Err(DebugProbeError::UnsupportedProtocol(protocol)),
        }
    }

    fn active_protocol(&self) -> Option<WireProtocol> {
        Some(WireProtocol::Swd)
    }

    fn has_arm_interface(&self) -> bool {
        true
    }

    fn try_as_dap_probe(&mut self) -> Option<&mut dyn DapProbe> {
        Some(self)
    }

    fn into_probe(self: Box<Self>) -> Box<dyn DebugProbe> {
        self
    }

    fn try_get_arm_debug_interface<'probe>(
        self: Box<Self>,
        sequence: Arc<dyn ArmDebugSequence>,
    ) -> Result<Box<dyn ArmDebugInterface + 'probe>, (Box<dyn DebugProbe>, ArmError)> {
        Ok(ArmCommunicationInterface::create(self, sequence, true))
    }
}

impl RawDapAccess for AirfrogProbe {
    fn raw_read_register(&mut self, address: RegisterAddress) -> Result<u32, ArmError> {
        // Flush any other pending operations first
        self.raw_flush()?;

        // Encode the command: [cmd][reg]
        let (cmd, reg) = match address {
            RegisterAddress::DpRegister(_) => (CMD_DP_READ, address.lsb()),
            RegisterAddress::ApRegister(_) => (CMD_AP_READ, address.lsb()),
        };

        // Send it
        let response = self.send_recv_read_airfrog(&[cmd, reg], 4)?.unwrap();

        // Received OK, convert to u32 and return
        let value = u32::from_le_bytes([response[0], response[1], response[2], response[3]]);
        Ok(value)
    }

    fn raw_write_register(&mut self, address: RegisterAddress, value: u32) -> Result<(), ArmError> {
        // Flush any other pending write block operations first
        self.flush_write_blocks()?;

        let write_reg = WriteReg {
            address,
            data: value,
        };
        self.write_regs.push(write_reg);

        Ok(())
    }

    fn raw_read_block(
        &mut self,
        address: RegisterAddress,
        values: &mut [u32],
    ) -> Result<(), ArmError> {
        // Flush any other pending operations first
        self.raw_flush()?;

        match address {
            // For DP registers, we don't have a bulk read API - as there's
            // little call for this operation.  Send individual reads.
            RegisterAddress::DpRegister(_) => {
                // No DP bulk - use individual reads
                for val in values {
                    *val = self.raw_read_register(address)?;
                }
                Ok(())
            }

            // We have a bulk AP read.  Send it.  We don't bother queuing
            // these up, as we assume probe-rs wants the data immediately.
            RegisterAddress::ApRegister(_) => {
                // Encode the command: [cmd][reg][count:2]
                let reg = address.lsb();
                let count = values.len() as u16;
                let mut command = vec![CMD_AP_BULK_READ, reg];
                command.extend_from_slice(&count.to_le_bytes());

                // Send it and read response: ([status])[count:2][data...]
                let response = self
                    .send_recv_read_airfrog(&command, 2 + (values.len() * 4))?
                    .unwrap();

                let returned_count = u16::from_le_bytes([response[0], response[1]]) as usize;
                if returned_count != values.len() {
                    return Err(ArmError::Probe(DebugProbeError::Other(format!(
                        "Count mismatch: expected {}, got {}",
                        values.len(),
                        returned_count
                    ))));
                }

                // Extract data and store in provided buffer
                for (i, val) in values.iter_mut().enumerate() {
                    let offset = 2 + (i * 4);
                    *val = u32::from_le_bytes([
                        response[offset],
                        response[offset + 1],
                        response[offset + 2],
                        response[offset + 3],
                    ]);
                }

                // Done
                Ok(())
            }
        }
    }

    fn raw_write_block(
        &mut self,
        address: RegisterAddress,
        values: &[u32],
    ) -> Result<(), ArmError> {
        // Flush any single register writes first
        self.flush_write_reg()?;

        match address {
            RegisterAddress::DpRegister(_) => {
                // Flush any other pending operations first
                self.flush_write_blocks()?;

                // There's no bulk DP write API as there's little use.  Use
                // individual writes instead
                for val in values {
                    self.raw_write_register(address, *val)?;
                }
            }
            RegisterAddress::ApRegister(_) => {
                // Queue up AP writes - they will be sent when `raw_flush()` is
                // called, which will also be if another type of register
                // operation (individual write, any sort of read) is executed
                let block = WriteBlock {
                    address,
                    data: values.to_vec(),
                };
                self.ap_write_blocks.push(block);
            }
        }
        Ok(())
    }

    fn raw_flush(&mut self) -> Result<(), ArmError> {
        self.flush_write_reg()?;
        self.flush_write_blocks()
    }

    fn configure_jtag(&mut self, _skip_scan: bool) -> Result<(), DebugProbeError> {
        Err(DebugProbeError::UnsupportedProtocol(WireProtocol::Jtag))
    }

    fn jtag_sequence(&mut self, _cycles: u8, _tms: bool, _tdi: u64) -> Result<(), DebugProbeError> {
        Err(DebugProbeError::UnsupportedProtocol(WireProtocol::Jtag))
    }

    fn swj_sequence(&mut self, _bit_len: u8, _bits: u64) -> Result<(), DebugProbeError> {
        self.target_reset()
    }

    fn swj_pins(
        &mut self,
        _pin_out: u32,
        _pin_select: u32,
        _pin_wait: u32,
    ) -> Result<u32, DebugProbeError> {
        Err(DebugProbeError::NotImplemented {
            function_name: "swj_pins",
        })
    }

    fn into_probe(self: Box<Self>) -> Box<dyn DebugProbe> {
        self as Box<dyn DebugProbe>
    }

    fn core_status_notification(&mut self, _state: CoreStatus) -> Result<(), DebugProbeError> {
        Ok(())
    }
}

impl DapProbe for AirfrogProbe {}
