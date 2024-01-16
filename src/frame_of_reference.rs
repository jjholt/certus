use std::marker::PhantomData;

use crate::{anatomy::BoneLocations, parse_csv::Coords};

use super::anatomy::{
    Landmark, Bone, Position, Side
};
use nalgebra::{Matrix3, Matrix4, Vector3, Vector4, Rotation3};
use itertools::izip;
// use super::change_frame::ChangeFrame;

pub trait Reference: Sized { }

// pub struct Global<'a> {
//     pub pin1: &'a [Matrix4<f64>],
//     pub pin2: &'a [Matrix4<f64>],
// }
#[derive(Debug)]
pub struct Global;
pub struct Tibia;
pub struct Femur;
#[derive(Debug)]
pub struct Patella;
pub struct Pin1;
pub struct Pin2;
pub struct Tracker;

pub struct UnitVectors {
    pub x: Vector3<f64>,
    pub y: Vector3<f64>,
    pub z: Vector3<f64>,
}

#[derive(Debug)]
pub struct CoordinateSystem<O, G> where O:Reference, G: Reference {
    pub origin: Vector4<f64>,
    pub transform: Matrix4<f64>,
    reference: PhantomData<O>,
    in_reference_to: PhantomData<G>,
}

pub trait Inverse <R: Reference, T: Reference>{
    fn inverse(&self) -> Option<Vec<CoordinateSystem<T,R>>> where Self: Sized;
}

pub trait ApplyTransform <O: Reference, G: Reference> {
    fn transform<X:Reference>(&self, matrix: &CoordinateSystem<G,X>, offset: &Vector4<f64>) -> Option<CoordinateSystem<O,X>>;
}

impl<O, G> ApplyTransform<O, G> for Option<CoordinateSystem<O,G>> where O: Reference, G: Reference {
    fn transform<X:Reference>(&self, matrix: &CoordinateSystem<G,X>, offset: &Vector4<f64>) -> Option<CoordinateSystem<O,X>> {
        self.as_ref().map(|this| this.apply::<X>(matrix, offset))
    }
}

impl <O, G> CoordinateSystem <O, G> where O: Reference, G: Reference{
    pub fn from_transform<N:Reference>(transform: Matrix4<f64>) -> CoordinateSystem< N, G> {
        let origin = transform.fixed_view::<4,1>(0, 3).into_owned();
        CoordinateSystem {
            origin,
            transform,
            reference: PhantomData,
            in_reference_to: PhantomData,
        }
    }
    pub fn change_frame<N>(&self, matrix: Option<&Vec<CoordinateSystem<G,N>>>, offset: &Vector4<f64>) -> Option<CoordinateSystem<O, N>> where N:Reference {
        matrix.as_ref().map(|matrix| self.apply(&matrix[0], offset))
    }
    pub fn tracker<N: Reference>(coords: Option<&Coords>) -> Option<Vec<CoordinateSystem<N, G>>> {
        let coords = coords?;
        let beep: Option<Vec<_>> = izip!(&coords.q0, &coords.qx, &coords.qy, &coords.qz)
            .map(Coords::rotation)
            .map(Matrix3::from)
            .enumerate()
            .map(|(i, rotation)| {
                let mut transform = Matrix4::identity();

                transform.fixed_view_mut::<3,3>(0, 0).copy_from(&rotation);
                transform.fixed_view_mut::<3,1>(0, 3).copy_from(&Vector3::new(coords.x[i], coords.y[i], coords.z[i]));
                Some(CoordinateSystem::<O, G>::from_transform::<N>(transform))
            })
            .collect();
            match beep {
                Some(n) => if n.is_empty() { None } else { Some(n) },
                None => None,
            }
    }
    pub fn floating_axis<N:Reference>(&self, other: &CoordinateSystem<N, G>) -> Vector3<f64> {
        self.unit_vectors().z.cross(&other.unit_vectors().x).normalize()
    }
    pub fn from_landmark(anatomy: &[Option<Landmark>]) -> Option<CoordinateSystem<O, Global>> {
        let (transform, point) = body_frame(anatomy)?;
        Some(CoordinateSystem {
            transform,
            origin: Vector4::new(point[0], point[1], point[2], 1.0),
            reference: PhantomData::<O>,
            in_reference_to: PhantomData::<Global>,
        })
    }
    pub fn rotation(&self, side: &Side) -> Vector3<f64> {
        let mut r = Matrix4::identity();
        r.fixed_view_mut::<4,3>(0, 0).copy_from(&self.transform.fixed_view::<4,3>(0, 0));
        let ang = Rotation3::from_matrix(&r.fixed_view::<3,3>(0, 0).into_owned());
        let ang = ang.euler_angles();
        match side {
            Side::Left => Vector3::new(-ang.0, -ang.1, -ang.2),
            Side::Right => Vector3::new(-ang.0, ang.1, ang.2),
        }
    }
    pub fn unit_vectors(&self) -> UnitVectors {
        let f = self.transform;
        UnitVectors {
            x: f.fixed_view::<3,1>(0, 0).into_owned(),
            y: f.fixed_view::<3,1>(0, 1).into_owned(),
            z: f.fixed_view::<3,1>(0, 2).into_owned(),
        }
    }
    pub fn apply<X: Reference>(&self, matrix: &CoordinateSystem<G,X>, offset: &Vector4<f64>) -> CoordinateSystem<O, X> {
        CoordinateSystem {
                origin: matrix.transform * (self.origin - offset),
                transform: matrix.transform * self.transform,
                reference: PhantomData,
                in_reference_to: PhantomData,
        }
    }
    pub fn inverse(&self) -> CoordinateSystem<G, O> {
        CoordinateSystem::<G,O>::from_transform(self.transform.try_inverse().unwrap())
    }
}


// impl <O: Reference, G: Reference> FrameChanger<O,G> for Option<CoordinateSystem<O, G>> {
//     fn change_frame<X: Reference>(&self) -> Option<ChangeFrame<O, X ,G>> {
//         match self {
//             Some(old) => Some(ChangeFrame::<O,X,G>::new(old)),
//             _ => None,
//         }
//     }
// }

impl<O, G> Inverse<O,G> for Option<Vec<CoordinateSystem<O, G>>>
where O:Reference, O: Reference, G: Reference
{
    fn inverse(&self) -> Option<Vec<CoordinateSystem<G, O>>> where Self: Sized {
        let invert = |f: &CoordinateSystem<O, G>| { Some(CoordinateSystem::<G,O>::from_transform(f.transform.try_inverse().unwrap()))};
        self.as_ref().and_then(|m| m.iter().map(invert).collect())
    }
}


fn body_frame(bone_locations: &[Option<Landmark>]) -> Option<(Matrix4<f64>, Vector3<f64>)> {
    let mut locations = BoneLocations::default(); 

    bone_locations.iter().for_each(|f| {
        if let Some(c) = f.as_ref() {
            match c.position[0] {
                Position::Medial => locations.medial = f.as_ref(),
                Position::Lateral => locations.lateral = f.as_ref(),
                Position::Posterior => locations.posterior = f.as_ref(),
                Position::Anterior => locations.anterior = f.as_ref(),
                Position::Distal => locations.distal = f.as_ref(),
                Position::Proximal => locations.proximal = f.as_ref(),
            }
        }
    });
    
    for location in bone_locations.iter().flatten() {
            match location.position[0] {
                Position::Medial => locations.medial = Some(location),
                Position::Lateral => locations.lateral = Some(location),
                Position::Distal => locations.distal = Some(location),
                Position::Posterior => locations.posterior = Some(location),
                Position::Anterior => locations.anterior = Some(location),
                Position::Proximal => locations.proximal = Some(location),
            }
    }
    let med = locations.medial
        .and_then(|f| f.data.probe.as_ref())
        .map(|f| f.mean_xyz());
    let lat = locations.lateral
        .and_then(|f| f.data.probe.as_ref())
        .map(|f| f.mean_xyz());

        let origin = (med? + lat?)/2.0;
        let i = match bone_locations[0].as_ref()?.side {
            Side::Right => (lat?-med?).normalize(),
            Side::Left => (med?-lat?).normalize(),
        };
        let tempk = match bone_locations[0].as_ref()?.bone {
            Bone::Tibia => origin - locations.distal.and_then(|f| f.data.probe.as_ref()).map(|f| f.mean_xyz())?,
            Bone::Femur => locations.proximal.and_then(|f| f.data.probe.as_ref()).map(|f| f.mean_xyz())?,
            Bone::Patella => todo!(),
        };

    let j = (tempk.normalize()).cross(&i);
    let k = i.cross(&j).normalize();

    let mut rot = Matrix4::identity();
    rot.fixed_view_mut::<3,3>(0, 0).copy_from(&Matrix3::from_columns(&[i, j, k]));

    let mut trans = Matrix4::identity();
    trans.fixed_view_mut::<3,1>(0, 3).copy_from(&origin);

    Some((trans*rot, origin))
}
// impl <'a> Reference for Global <'a> { }
impl Reference for Global { }
impl Reference for Tibia { }
impl Reference for Femur { }
impl Reference for Patella { }
impl Reference for Pin1 { }
impl Reference for Pin2 { }
impl Reference for Tracker { }
