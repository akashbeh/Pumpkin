use pumpkin_util::math::{boundingbox::BoundingBox, vector3::Vector3};

#[derive(Clone, Copy, Debug)]
pub struct CollisionShape {
    pub min: Vector3<f64>,
    pub max: Vector3<f64>,
}

impl CollisionShape {
    pub fn is_empty() -> bool {
        unimplemented!()
    }

    pub fn to_box(&self) -> BoundingBox {
        BoundingBox {
            min: self.min,
            max: self.max,
        }
    }
}
