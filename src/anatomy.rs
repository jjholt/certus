use std::{error::Error, fmt, ffi};
use super::parse_csv::*;
pub type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Debug)]
#[allow(dead_code)]
pub enum Position {
    Medial,
    Lateral,
    Posterior,
    Anterior,
    Distal,
    Proximal,
}

#[allow(dead_code)]
#[derive(Debug)]
pub enum Side {
    Left,
    Right,
}
#[allow(dead_code)]
#[derive(Debug)]
pub enum Bone {
    Tibia,
    Femur,
    Patella,
}
#[allow(dead_code)]
pub struct LandmarkSetup <'a>{
    folder: &'a str,
    tests: &'a[ffi::OsString],
    side: &'a Side,
}

#[derive(Debug)]
pub struct Landmark <'a> {
    pub position: &'a [Position],
    pub side: &'a Side,
    pub bone: Bone,
    pub data: Structure,
    pub header: Header,
}

#[allow(dead_code)]
#[derive(Default)]
pub struct BoneLocations <'a> {
    pub medial: Option<&'a Landmark<'a>>,
    pub lateral: Option<&'a Landmark<'a>>,
    pub proximal: Option<&'a Landmark<'a>>,
    pub distal: Option<&'a Landmark<'a>>,
    pub posterior: Option<&'a Landmark<'a>>,
    pub anterior: Option<&'a Landmark<'a>>,
}

#[allow(dead_code)]
impl Position {
    fn filename(&self) -> &str {
        match self {
            Position::Medial => "M",
            Position::Lateral => "L",
            Position::Posterior => "P",
            Position::Anterior => "A",
            Position::Distal => "D",
            Position::Proximal => "P",
        }
    }
}

#[allow(dead_code)]
impl Bone {
    fn filename(&self) -> &str {
        match self {
            Bone::Tibia => "T",
            Bone::Femur => "F",
            Bone::Patella => "P",
        }
    }
}

impl fmt::Display for Position {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}", self)
    }
}

#[allow(dead_code)]
impl <'a> LandmarkSetup <'a>{
    pub fn new(folder: &'a str, tests: &'a[ffi::OsString], side: &'a Side) -> Self {
        Self {
            folder,
            tests,
            side,
        }
    }
    pub fn create(&'a self, bone: Bone, position: &'a [Position]) -> Landmark {
        let position_names: String = position.iter().map(|p| p.filename().to_string()).collect();
        let filename = format!("{}/{}{}.csv", self.folder, bone.filename(), position_names);
        let csv = Csv::new(&filename).ok().unwrap();
        let data = csv.content;
        let header = csv.header;
        Landmark {
            side: self.side,
            position,
            bone,
            data,
            header,
        }
    }
    pub fn raw_csv(&self, name: &str) -> Result<Csv> {
        let name = format!("{}/{}", self.folder, name);
        println!("Reading csv from:{}", name);
        Csv::new(&name)
    }
}

impl <'a> Landmark<'a> {
    pub fn pin1(&self) -> &Coords {
        &self.data.pin1
    }
    pub fn pin2(&self) -> &Coords {
        &self.data.pin2
    }
    pub fn new(folder: &'a str, tests: &'a[ffi::OsString], side: &'a Side) -> LandmarkSetup<'a> {
        LandmarkSetup {
            folder,
            tests,
            side,
        }
    }
}
