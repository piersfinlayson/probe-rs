//! airfrog driver - see https://piers.rocks/u/airfrog

use std::io::{Read, Write};

use crate::{
    CoreStatus,
    architecture::arm::{
        ArmError, RawDapAccess, RegisterAddress,
        communication_interface::DapProbe,
    },
    probe::{
        DebugProbe, DebugProbeError, DebugProbeInfo, DebugProbeSelector,
        ProbeCreationError, ProbeError, ProbeFactory, WireProtocol,
        BoxedProbeError,
    },
};
use std::sync::Arc;
use crate::architecture::arm::{ArmDebugInterface, sequences::ArmDebugSequence, ArmCommunicationInterface};

/// Airfrog probe factory
#[derive(Debug)]
pub struct AirfrogFactory;

/// Airfrog probe implementation
#[derive(Debug)]
pub struct AirfrogProbe {
    /// Host address 
    host: String,
    /// Port number
    port: u16,
    /// Current SWD speed in kHz
    current_speed_khz: u32,
    /// Whether we're attached to the target
    attached: bool,
    /// TCP stream for binary protocol
    stream: Option<std::net::TcpStream>,
}

/// Airfrog-specific errors
#[derive(thiserror::Error, Debug)]
pub enum AirfrogError {
    /// IO error  
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
    /// Invalid response from airfrog
    #[error("Invalid response: {0}")]
    InvalidResponse(String),
    /// Airfrog API error
    #[error("Airfrog API error: {0}")]
    ApiError(String),
    /// Invalid URL
    #[error("Invalid IP/port: {0}")]
    InvalidUrl(String),
}

impl ProbeError for AirfrogError {}

impl std::fmt::Display for AirfrogFactory {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "Airfrog SWD Probe")
    }
}

impl AirfrogProbe {
    /// Create a new airfrog probe
    pub fn new(address: String) -> Result<Self, AirfrogError> {
        let (host, port) = if let Some((h, p)) = address.split_once(':') {
            let port = p.parse::<u16>()
                .map_err(|_| AirfrogError::InvalidUrl(format!("Invalid port: {}", p)))?;
            (h.to_string(), port)
        } else {
            // Default to port 4146 if not specified
            (address, 4146)
        };

        Ok(Self {
            host,
            port,
            current_speed_khz: 1000,
            attached: false,
            stream: None,
        })
    }

    /// Convert kHz to airfrog Speed enum
    fn khz_to_speed(khz: u32) -> AirfrogSpeed {
        //println!("Converting kHz {} to Airfrog speed", khz);
        match khz {
            0..=500 => AirfrogSpeed::Slow,
            501..=1000 => AirfrogSpeed::Medium,
            1001..=2000 => AirfrogSpeed::Fast,
            _ => AirfrogSpeed::Turbo,
        }
    }

    /// Convert airfrog Speed enum to kHz
    fn speed_to_khz(speed: AirfrogSpeed) -> u32 {
        //println!("Converting Airfrog speed {:?} to kHz", speed);
        match speed {
            AirfrogSpeed::Slow => 500,
            AirfrogSpeed::Medium => 1000,
            AirfrogSpeed::Fast => 2000,
            AirfrogSpeed::Turbo => 4000,
        }
    }
}

// Binary protocol helpers
impl AirfrogProbe {
    fn send_command(&mut self, data: &[u8]) -> Result<(), AirfrogError> {
        let stream = self.stream.as_mut()
            .ok_or_else(|| AirfrogError::ApiError("Not connected".to_string()))?;
        stream.write_all(data)?;
        Ok(())
    }

    fn read_response(&mut self, len: usize) -> Result<Vec<u8>, AirfrogError> {
        let stream = self.stream.as_mut()
            .ok_or_else(|| AirfrogError::ApiError("Not connected".to_string()))?;
        let mut buf = vec![0u8; len];
        stream.read_exact(&mut buf)?;
        Ok(buf)
    }

    fn to_arm_error(e: AirfrogError) -> ArmError {
        ArmError::Probe(DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(e))))
    }
    
    fn reset_target_impl(&mut self) -> Result<(), DebugProbeError> {
        self.send_command(&[0xF1])
            .map_err(|e| DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(e))))?;
        
        let response = self.read_response(1)
            .map_err(|e| DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(e))))?;
        
        if response[0] != 0x00 {
            return Err(DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(
                AirfrogError::ApiError("Reset failed".to_string())
            ))));
        }
        
        Ok(())
    }
}

/// Airfrog speed enumeration (matches your REST API)
// Add serde derives to AirfrogSpeed
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
enum AirfrogSpeed {
    Slow,
    Medium, 
    Fast,
    Turbo,
}

impl std::fmt::Display for AirfrogSpeed {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            AirfrogSpeed::Slow => write!(f, "Slow"),
            AirfrogSpeed::Medium => write!(f, "Medium"),
            AirfrogSpeed::Fast => write!(f, "Fast"),
            AirfrogSpeed::Turbo => write!(f, "Turbo"),
        }
    }
}

impl ProbeFactory for AirfrogFactory {
    fn list_probes(&self) -> Vec<DebugProbeInfo> {
        //println!("Listing Airfrog probes");
        vec![]
    }

    // Accept any probe whose serial number starts with "airfrog:",
    // irrespective of the VID:PID.
    fn list_probes_filtered(&self, selector: Option<&DebugProbeSelector>) -> Vec<DebugProbeInfo> {
        //println!("Listing Airfrog probes with selector: {:?}", selector);
        let Some(selector) = selector else {
            return self.list_probes();
        };

        let Some(serial) = selector.serial_number.as_deref() else {
            return vec![];
        };

        // Check if it starts with "airfrog:"
        if let Some(address) = serial.strip_prefix("airfrog:") {
            //println!("Found Airfrog probe {:x}:{:x} with address: {}", selector.vendor_id, selector.product_id, address);
            vec![DebugProbeInfo::new(
                format!("Airfrog SWD Probe ({})", address),
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
        //println!("Opening Airfrog probe with selector: {:?}", selector);
        // Validate this is actually ours
        if (selector.vendor_id, selector.product_id) != (0, 0) {
            return Err(DebugProbeError::ProbeCouldNotBeCreated(ProbeCreationError::NotFound));
        }
        
        let serial = selector.serial_number.as_ref()
            .ok_or_else(|| DebugProbeError::ProbeCouldNotBeCreated(ProbeCreationError::NotFound))?;

        //println!("Serial number for Airfrog probe: {}", serial);
        
        let address = serial.strip_prefix("airfrog:")
            .ok_or_else(|| DebugProbeError::ProbeCouldNotBeCreated(ProbeCreationError::NotFound))?;

        //println!("Parsed Airfrog address: {}", address);
        
        let probe = AirfrogProbe::new(address.to_string())
            .map_err(|e| DebugProbeError::ProbeCouldNotBeCreated(ProbeCreationError::from(e)))?;
        
        //println!("Successfully created Airfrog probe at {}", probe.base_url);
        Ok(Box::new(probe))
    }
}

impl DebugProbe for AirfrogProbe {
    fn get_name(&self) -> &str {
        //println!("Getting Airfrog probe name");
        "Airfrog SWD Probe - https://piers.rocks/u/airfrog"
    }

    fn speed_khz(&self) -> u32 {
        //println!("Getting current speed: {} kHz", self.current_speed_khz);
        self.current_speed_khz
    }

    fn set_speed(&mut self, speed_khz: u32) -> Result<u32, DebugProbeError> {
        let speed = Self::khz_to_speed(speed_khz);
        self.current_speed_khz = Self::speed_to_khz(speed);
        Ok(self.current_speed_khz)
    }

    fn attach(&mut self) -> Result<(), DebugProbeError> {
        let addr = format!("{}:{}", self.host, self.port);
        let mut stream = std::net::TcpStream::connect(&addr)
            .map_err(|e| DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(
                AirfrogError::Io(e)
            ))))?;
        
        // Binary API handshake - send version
        stream.write_all(&[0x01])
            .map_err(|e| DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(
                AirfrogError::Io(e)
            ))))?;
        
        // Read version ack
        let mut version_buf = [0u8; 1];
        stream.read_exact(&mut version_buf)
            .map_err(|e| DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(
                AirfrogError::Io(e)
            ))))?;
        
        if version_buf[0] != 0x01 {
            return Err(DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(
                AirfrogError::ApiError("Version mismatch".to_string())
            ))));
        }
        
        self.stream = Some(stream);
        self.attached = true;
        
        // Reset target using binary protocol
        self.send_command(&[0xF1])
            .map_err(|e| DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(e))))?;
        
        let response = self.read_response(1)
            .map_err(|e| DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(e))))?;
        
        if response[0] != 0x00 {
            return Err(DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(
                AirfrogError::ApiError("Reset failed".to_string())
            ))));
        }
        
        Ok(())
    }

    fn detach(&mut self) -> Result<(), crate::Error> {
        //println!("Detaching from target");
        self.attached = false;
        self.stream = None;
        Ok(())
    }

    fn target_reset(&mut self) -> Result<(), DebugProbeError> {
        self.reset_target_impl()
    }
    
    fn target_reset_assert(&mut self) -> Result<(), DebugProbeError> {
        //println!("Asserting target reset");
        Err(DebugProbeError::NotImplemented {
            function_name: "target_reset_assert",
        })
    }

    fn target_reset_deassert(&mut self) -> Result<(), DebugProbeError> {
        //println!("Deasserting target reset");
        Err(DebugProbeError::NotImplemented {
            function_name: "target_reset_deassert", 
        })
    }

    fn select_protocol(&mut self, protocol: WireProtocol) -> Result<(), DebugProbeError> {
        //println!("Selecting protocol: {:?}", protocol);
        match protocol {
            WireProtocol::Swd => Ok(()),
            WireProtocol::Jtag => Err(DebugProbeError::UnsupportedProtocol(protocol)),
        }
    }

    fn active_protocol(&self) -> Option<WireProtocol> {
        //println!("Getting active protocol");
        Some(WireProtocol::Swd)
    }

    fn has_arm_interface(&self) -> bool {
        //println!("Checking if Airfrog has ARM interface");
        true
    }

    fn try_as_dap_probe(&mut self) -> Option<&mut dyn DapProbe> {
        //println!("Trying to get as DAP probe");
        Some(self)
    }

    fn into_probe(self: Box<Self>) -> Box<dyn DebugProbe> {
        //println!("Converting Airfrog probe into Box<dyn DebugProbe>");
        self
    }

    fn try_get_arm_debug_interface<'probe>(
        self: Box<Self>,
        sequence: Arc<dyn ArmDebugSequence>,
    ) -> Result<Box<dyn ArmDebugInterface + 'probe>, (Box<dyn DebugProbe>, ArmError)> {
        //println!("Creating ARM debug interface for Airfrog probe {:?}", sequence);
        Ok(ArmCommunicationInterface::create(self, sequence, true))
    }
}

impl RawDapAccess for AirfrogProbe {
    fn raw_read_register(&mut self, address: RegisterAddress) -> Result<u32, ArmError> {
        let (cmd, reg) = match address {
            RegisterAddress::DpRegister(_) => (0x00, address.lsb() as u8), // DP_READ
            RegisterAddress::ApRegister(_) => (0x02, address.lsb() as u8), // AP_READ  
        };
        
        // Send: [cmd][reg]
        self.send_command(&[cmd, reg]).map_err(Self::to_arm_error)?;
        
        // Read: [status][data:4] 
        let response = self.read_response(5).map_err(Self::to_arm_error)?;
        
        if response[0] != 0x00 {
            return Err(ArmError::Probe(DebugProbeError::Other(
                format!("Register read failed, status: 0x{:02X}", response[0])
            )));
        }
        
        let value = u32::from_le_bytes([response[1], response[2], response[3], response[4]]);
        Ok(value)
    }

    fn raw_write_register(&mut self, address: RegisterAddress, value: u32) -> Result<(), ArmError> {
        let (cmd, reg) = match address {
            RegisterAddress::DpRegister(_) => (0x01, address.lsb() as u8), // DP_WRITE
            RegisterAddress::ApRegister(_) => (0x03, address.lsb() as u8), // AP_WRITE
        };
        
        // Send: [cmd][reg][data:4]
        let mut command = vec![cmd, reg];
        command.extend_from_slice(&value.to_le_bytes());
        self.send_command(&command).map_err(Self::to_arm_error)?;
        
        // Read: [status]
        let response = self.read_response(1).map_err(Self::to_arm_error)?;
        
        if response[0] != 0x00 {
            return Err(ArmError::Probe(DebugProbeError::Other(
                format!("Register write failed, status: 0x{:02X}", response[0])
            )));
        }
        
        Ok(())
    }

    fn raw_read_block(&mut self, address: RegisterAddress, values: &mut [u32]) -> Result<(), ArmError> {
        match address {
            RegisterAddress::DpRegister(_) => {
                // No DP bulk - use individual reads
                for val in values {
                    *val = self.raw_read_register(address)?;
                }
                Ok(())
            }
            RegisterAddress::ApRegister(_) => {
                let reg = address.lsb() as u8;
                let count = values.len() as u16;
                
                // Send: [0x12][reg][count:2]
                let mut command = vec![0x12, reg];
                command.extend_from_slice(&count.to_le_bytes());
                self.send_command(&command).map_err(Self::to_arm_error)?;
                
                // Read: [status][count:2][data...]
                let response_len = 3 + (values.len() * 4);
                let response = self.read_response(response_len).map_err(Self::to_arm_error)?;
                
                if response[0] != 0x00 {
                    return Err(ArmError::Probe(DebugProbeError::Other(
                        format!("Bulk read failed, status: 0x{:02X}", response[0])
                    )));
                }
                
                let returned_count = u16::from_le_bytes([response[1], response[2]]) as usize;
                if returned_count != values.len() {
                    return Err(ArmError::Probe(DebugProbeError::Other(
                        format!("Count mismatch: expected {}, got {}", values.len(), returned_count)
                    )));
                }
                
                // Extract data
                for (i, val) in values.iter_mut().enumerate() {
                    let offset = 3 + (i * 4);
                    *val = u32::from_le_bytes([
                        response[offset], 
                        response[offset + 1], 
                        response[offset + 2], 
                        response[offset + 3]
                    ]);
                }
                
                Ok(())
            }
        }
    }

    fn raw_write_block(
        &mut self,
        address: RegisterAddress,
        values: &[u32],
    ) -> Result<(), ArmError> {
        println!("Writing block to address: {:?} {}", address, values.len());
        // Default implementation - could be optimized later
        for val in values {
            self.raw_write_register(address, *val)?;
        }
        Ok(())
    }

    fn raw_flush(&mut self) -> Result<(), ArmError> {
        //println!("Flushing raw DAP access");
        // HTTP is always flushed
        Ok(())
    }

    fn configure_jtag(&mut self, _skip_scan: bool) -> Result<(), DebugProbeError> {
        //println!("Configuring JTAG - not supported in Airfrog");
        Err(DebugProbeError::UnsupportedProtocol(WireProtocol::Jtag))
    }

    fn jtag_sequence(&mut self, _cycles: u8, _tms: bool, _tdi: u64) -> Result<(), DebugProbeError> {
        //println!("JTAG sequence - not supported in Airfrog");
        Err(DebugProbeError::UnsupportedProtocol(WireProtocol::Jtag))
    }
    
    fn swj_sequence(&mut self, _bit_len: u8, _bits: u64) -> Result<(), DebugProbeError> {
        // SWJ sequence typically used for line reset - use target reset
        self.reset_target_impl()
    }

    fn swj_pins(
        &mut self,
        _pin_out: u32,
        _pin_select: u32, 
        _pin_wait: u32,
    ) -> Result<u32, DebugProbeError> {
        //println!("SWJ pins - not supported in Airfrog");
        // Pin manipulation - not supported
        Err(DebugProbeError::NotImplemented {
            function_name: "swj_pins",
        })
    }

    fn into_probe(self: Box<Self>) -> Box<dyn DebugProbe> {
        //println!("Converting Airfrog probe into Box<dyn DebugProbe>");
        self as Box<dyn DebugProbe>
    }

    fn core_status_notification(&mut self, _state: CoreStatus) -> Result<(), DebugProbeError> {
        //println!("Core status notification - not implemented for Airfrog");
        // Not needed for HTTP probe
        Ok(())
    }
}

impl DapProbe for AirfrogProbe {}
