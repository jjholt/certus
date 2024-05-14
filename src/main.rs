use std::{env, process, fs, ffi};

mod anatomy;
mod parse_csv;
mod frame_of_reference;
mod live_plot;
mod output;

// use change_frame::TryWith;
use output::*;
use frame_of_reference::{Inverse, CoordinateSystem, Tibia, Femur, Global, Pin1, Pin2, Patella, ApplyTransform};
use anatomy::{
    Result, Side, Bone, Position, Landmark
};
use csv::Writer;
use nalgebra::Vector4;

fn main() -> Result<()> {
    let no_offset = Vector4::zeros();
    let (folder, side) = get_args();
    // let tests = ["FE1", "FE2"];
    let tests = get_test_files(&folder)?;

    let landmark = Landmark::new(&folder, &tests, &side);

    // Import digitisation data
    let tibia_landmarks = &[
        landmark.create(Bone::Tibia, &[Position::Medial]),
        landmark.create(Bone::Tibia, &[Position::Lateral]),
        landmark.create(Bone::Tibia, &[Position::Distal]),
    ];

    let femur_landmarks = &[
        landmark.create(Bone::Femur, &[Position::Medial]),
        landmark.create(Bone::Femur, &[Position::Lateral]),
        landmark.create(Bone::Femur, &[Position::Proximal]),
    ];
    let patella_landmarks = &[
        landmark.create(Bone::Patella, &[Position::Medial, Position::Proximal]),
        landmark.create(Bone::Patella, &[Position::Medial, Position::Distal]),
        landmark.create(Bone::Patella, &[Position::Lateral, Position::Proximal]),
        landmark.create(Bone::Patella, &[Position::Lateral, Position::Distal]),
    ];
    loaded_landmarks(&[tibia_landmarks, femur_landmarks, patella_landmarks]);

    // Define bone coordinate systems
    let cs_tibia: Option<CoordinateSystem<Tibia, Global>> = CoordinateSystem::<Tibia,Global>::from_landmark(tibia_landmarks);
    let cs_femur = CoordinateSystem::<Femur, Global>::from_landmark(femur_landmarks);
    let cs_patella = CoordinateSystem::<Patella, Global>::from_landmark(patella_landmarks);

    // Define tracker coordinate systems
    let cs_pin1: Option<Vec<CoordinateSystem<Pin1, Global>>> = CoordinateSystem::<Pin1, Global>::tracker::<Pin1>(tibia_landmarks[0].as_ref().and_then(|f| f.pin1()));
    let cs_pin2 = CoordinateSystem::<Pin2, Global>::tracker::<Pin2>(femur_landmarks[0].as_ref().and_then(|f| f.pin2()));
    let cs_tracker_patella = CoordinateSystem::<Patella, Global>::tracker::<Patella>(patella_landmarks[0].as_ref().and_then(|f| f.patella()));

    // Calculate bone-to-tracker transform
    let tibia_in_pin1 = cs_tibia
        .and_then(|f| f.change_frame(cs_pin1.inverse().as_ref(), &no_offset));
    let femur_in_pin2 = cs_femur
        .and_then(|f| f.change_frame(cs_pin2.inverse().as_ref(), &no_offset));
    let patella_in_patella_tracker = cs_patella
        .and_then(|f| f.change_frame(cs_tracker_patella.inverse().as_ref(), &no_offset));
    
    // let mut tf_angles: Vec<Vector3<f64>> = vec![];
    // let mut tf_translations: Vec<Vector3<f64>> = vec![];

    for test in &tests {
        let path = format!("output/{}", &folder);
        let filename = format!("{}/{}", &path, test.to_string_lossy());
        fs::create_dir_all(path)?;
        let mut wtr = Writer::from_path(&filename)?;

        // Import tracked data
        let csv = landmark.raw_csv(&test.to_string_lossy())?;

        // Define dynamic tracker positions
        let global_ti_pin1 = CoordinateSystem::<Pin1, Global>::tracker(csv.content.pin1.as_ref());
        let global_ti_pin2 = CoordinateSystem::<Pin2, Global>::tracker(csv.content.pin2.as_ref());
        let global_ti_patella = CoordinateSystem::<Patella, Global>::tracker(csv.content.patella.as_ref());

        //Calculate dynamic bone positions
        for (i, _) in global_ti_pin1.as_ref().unwrap().iter().enumerate() {
            let tibia_in_global = global_ti_pin1.as_ref().and_then(|f| tibia_in_pin1.transform(&f[i], &no_offset));
            let femur_in_global = global_ti_pin2.as_ref().and_then(|f| femur_in_pin2.transform(&f[i], &no_offset));
            let patella_in_global = global_ti_patella.as_ref().and_then(|f| patella_in_patella_tracker.transform(&f[i], &no_offset));

            // Serialise the output for the csv
            if let (Some(t), Some(f)) = (tibia_in_global, femur_in_global) {
                let record = Output::new(&t, &f, &side);
                wtr.serialize(&record)?;
            }

            // tf_angles.push(tf_ang);
            // tf_translations.push(tf_transl);
        }
        wtr.flush()?;
    }

    Ok(())
}

fn get_args() -> (String, Side) {
    let mut args = env::args();
    args.next(); // Program name
    let folder = args.next().unwrap_or_else(|| {
        eprintln!("No root folder provided"); process::exit(1)
    });
    let side = match args.next() {
        Some(arg) => {
            let arg = arg.to_lowercase();
            if arg == "r" || arg == "right" {
                Side::Right
            } else if arg == "l" || arg == "left" {
                Side::Left
            } else {
                eprintln!("Valid options are l/left or r/right"); process::exit(1)
            }
        },
        None => {
            eprintln!("Define a side with either r/right or l/left");
            process::exit(1);
        }
    };
    (folder, side)
}

fn loaded_landmarks(arr: &[&[Option<Landmark>]]) {
    for c in arr {
        if let Some(v) = c[0].as_ref() {
            println!("Loaded: {:#?}", v.bone);
        } 
    }
}

fn get_test_files(root: &str) -> Result<Vec<ffi::OsString>> {
    let result = fs::read_dir(root)?
        .map(|f| f.unwrap().path())
        .map(|f| f.file_name().unwrap().to_owned())
        .filter(|f| f.to_string_lossy().contains("FE"))
        .collect();

    Ok(result)
}
