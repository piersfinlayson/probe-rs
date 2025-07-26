//! airfrog driver - see https://piers.rocks/u/airfrog

use std::time::Duration;

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
use ureq::Agent;

/// Airfrog probe factory
#[derive(Debug)]
pub struct AirfrogFactory;

/// Airfrog probe implementation
#[derive(Debug)]
pub struct AirfrogProbe {
    /// Base URL for the airfrog device (e.g., "http://192.168.0.103")
    base_url: String,
    /// Current SWD speed in kHz
    current_speed_khz: u32,
    /// Whether we're attached to the target
    attached: bool,
    /// HTTP agent for making requests
    agent: Option<Agent>,
}

/// Airfrog-specific errors
#[derive(thiserror::Error, Debug)]
pub enum AirfrogError {
    /// HTTP request failed
    #[error("HTTP request failed: {0}")]
    Http(#[from] Box<ureq::Error>),    /// Invalid response from airfrog
    #[error("Invalid response: {0}")]
    InvalidResponse(String),
    /// Airfrog API error
    #[error("Airfrog API error: {0}")]
    ApiError(String),
    /// Invalid URL
    #[error("Invalid URL: {0}")]
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
    pub fn new(base_url: String) -> Result<Self, AirfrogError> {
        if !base_url.starts_with("http://") && !base_url.starts_with("https://") {
            return Err(AirfrogError::InvalidUrl(format!(
                "URL must start with http:// or https://, got: {}", base_url
            )));
        }


        Ok(Self {
            base_url,
            current_speed_khz: 1000,
            attached: false,
            agent: None,
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

    /// Make a GET request to the airfrog API
    fn get(&self, endpoint: &str) -> Result<serde_json::Value, AirfrogError> {
        let url = format!("{}{}", self.base_url, endpoint);
        let agent = self.agent.as_ref()
            .ok_or_else(|| AirfrogError::ApiError("HTTP agent not initialized".to_string()))?;
        let response = agent.get(&url)
            .timeout(Duration::from_secs(10))
            .call()
            .map_err(|e| AirfrogError::Http(e.into()))?;
        
        let json: serde_json::Value = response.into_json()
            .map_err(|e| AirfrogError::InvalidResponse(format!("JSON parse error: {}", e)))?;
        Ok(json)
    }

    /// Make a POST request to the airfrog API
    fn post(&self, endpoint: &str, body: Option<serde_json::Value>) -> Result<serde_json::Value, AirfrogError> {
        let url = format!("{}{}", self.base_url, endpoint);
        let agent = self.agent.as_ref()
            .ok_or_else(|| AirfrogError::ApiError("HTTP agent not initialized".to_string()))?;
        let request = agent.post(&url).timeout(Duration::from_secs(10));

        let response = if let Some(body) = body {
            request.send_json(body).map_err(|e| AirfrogError::Http(e.into()))?
        } else {
            request.call().map_err(|e| AirfrogError::Http(e.into()))?
        };
        
        let json: serde_json::Value = response.into_json()
            .map_err(|e| AirfrogError::InvalidResponse(format!("JSON parse error: {}", e)))?;
        Ok(json)
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
        
        let base_url = format!("http://{}", address);
        
        let probe = AirfrogProbe::new(base_url)
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
        let body = serde_json::json!({"speed": speed.to_string()});

        self.post("/api/target/config/speed", Some(body))
            .map_err(|e| DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(e))))?;
        
        self.current_speed_khz = Self::speed_to_khz(speed);
        Ok(self.current_speed_khz)
    }

    fn attach(&mut self) -> Result<(), DebugProbeError> {
        self.agent = Some(ureq::Agent::new());
        self.post("/api/target/reset", None)
            .map_err(|e| DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(e))))?;
        
        self.attached = true;
        Ok(())
    }

    fn detach(&mut self) -> Result<(), crate::Error> {
        //println!("Detaching from target");
        self.attached = false;
        self.agent = None;
        Ok(())
    }

    fn target_reset(&mut self) -> Result<(), DebugProbeError> {
        self.post("/api/target/reset", None)
            .map_err(|e| DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(e))))?;
        
        Ok(())
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
        let endpoint = match address {
            RegisterAddress::DpRegister(_) => {
                format!("/api/raw/dp/read/0x{:X}", address.lsb())
            }
            RegisterAddress::ApRegister(_) => {
                format!("/api/raw/ap/read/0x0/0x{:X}", address.lsb())
            }
        };

        //println!("Reading register using endpoint: {}", endpoint);
        
        let response = match self.get(&endpoint) {
            Ok(response) => response,
            Err(e) => {
                //println!("Failed to read register: {:?}", e);
                return Err(ArmError::Probe(DebugProbeError::Other(format!("HTTP request failed: {}", e))));
            },
        };
        
        let data_str = match response["data"].as_str() {
            Some(data) => data,
            None => {
                //println!("Invalid response format: {:?}", response);
                return Err(ArmError::Probe(DebugProbeError::Other("Invalid response format".into())));
            }
        };

        let value = match u32::from_str_radix(data_str.trim_start_matches("0x"), 16) {
            Ok(value) => value,
            Err(_) => {
                //println!("Invalid hex value: {}", data_str);
                return Err(ArmError::Probe(DebugProbeError::Other("Invalid hex value".into())));
            },
        };

        //println!("Read register at address: {address:?} {value:#010X}");

        Ok(value)
    }

    fn raw_write_register(&mut self, address: RegisterAddress, value: u32) -> Result<(), ArmError> {
        //println!("Writing register at address: {address:?} {value:010X}");
        let endpoint = match address {
            RegisterAddress::DpRegister(_) => {
                format!("/api/raw/dp/write/0x{:X}", address.lsb())
            }
            RegisterAddress::ApRegister(_) => {
                format!("/api/raw/ap/write/0x0/0x{:X}", address.lsb())
            }
        };

        let body = serde_json::json!({"data": format!("0x{:08X}", value)});

        //println!("Writing register using endpoint: {} body: {}", endpoint, body);

        //self.post(&endpoint, Some(body))
        //    .map_err(|_| ArmError::Probe(DebugProbeError::Other("HTTP request failed".into())))?;
        match self.post(&endpoint, Some(body)) {
            Ok(_) => Ok(()),
            Err(e) => {
                //println!("Failed {e:?}");
                Err(ArmError::Probe(DebugProbeError::Other(format!("HTTP request failed: {}", e))))
            },
        }
        
        //Ok(())
    }

    fn raw_read_block(
        &mut self,
        address: RegisterAddress,
        values: &mut [u32],
    ) -> Result<(), ArmError> {
        println!("Reading block from address: {:?} {}", address, values.len());
        match address {
            RegisterAddress::DpRegister(_) => {
                for val in values {
                    *val = self.raw_read_register(address)?;
                }
            }
            RegisterAddress::ApRegister(_) => {
                let endpoint = format!("/api/raw/ap/bulk/read/0x0/0x{}", address.lsb());
                let body = serde_json::json!({
                    "count": values.len()
                });
                
                let response = match self.post(&endpoint, Some(body)) {
                    Ok(response) => response,
                    Err(e) => {
                        println!("Failed to read block: {:?}", e);
                        return Err(ArmError::Probe(DebugProbeError::Other(format!("HTTP request failed: {}", e))));
                    }
                };
                
                // Expecting {"data": ["0x12345678", "0x87654321", ...]}
                let data_array = match response["data"].as_array() {
                    Some(array) => array,
                    None => {
                        println!("Invalid response format: {:?}", response);
                        return Err(ArmError::Probe(DebugProbeError::Other("Missing data array".into())));
                    }
                };
                
                if data_array.len() != values.len() {
                    println!("Expected {} values, got {}", values.len(), data_array.len());
                    return Err(ArmError::Probe(DebugProbeError::Other(format!(
                        "Expected {} values, got {}", values.len(), data_array.len()))));
                }
                
                for (i, val_json) in data_array.iter().enumerate() {
                    let val_str = match val_json.as_str() {
                        Some(s) => s,
                        None => {
                            println!("Invalid value format at index {}: {:?}", i, val_json);
                            return Err(ArmError::Probe(DebugProbeError::Other("Invalid value format".into())));
                        }
                    };
                    values[i] = u32::from_str_radix(val_str.trim_start_matches("0x"), 16)
                        .map_err(|_| {
                            println!("Invalid hex value: {}", val_str);
                            ArmError::Probe(DebugProbeError::Other("Invalid hex value".into()))
                        })?;
                }
            }
        }

        Ok(())
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
        //println!("SWJ sequence");
        self.post("/api/raw/reset", None)
            .map(|_| ())
            .map_err(|e| DebugProbeError::ProbeSpecific(BoxedProbeError(Box::new(e))))
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
