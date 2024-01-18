use piston_window::{EventLoop, PistonWindow, WindowSettings};
use plotters::prelude::*;
use plotters_piston::draw_piston_window;

use std::collections::vec_deque::VecDeque;

use crate::output::Output;

const FPS: u32 = 10;
const LENGTH: u32 = 20;
const N_DATA_POINTS: usize = (FPS * LENGTH) as usize;

pub struct Plot {
    window: PistonWindow,
    epoch: usize,
    data: Vec<VecDeque<f32>>,
}

impl Plot {
    pub fn new() -> Plot {
        let mut window: PistonWindow = WindowSettings::new("Real Time Sine Wave", [450, 300])
            .samples(4)
            .build()
            .unwrap();
        window.set_max_fps(FPS as u64);
        Self {
            window,
            epoch: 0,
            data: vec![VecDeque::from(vec![0f32; N_DATA_POINTS+1])],
        }
    }
    pub fn plot(&mut self, record: &Output) {
        draw_piston_window(&mut self.window, |b| {
            let root = b.into_drawing_area();
            root.fill(&WHITE)?;

            if self.data[0].len() == N_DATA_POINTS + 1 {
                self.data[0].pop_front();
            }
            self.data[0].push_back(record.flexion as f32);

            let mut cc = ChartBuilder::on(&root)
                .margin(10)
                .caption("Flexion angle", ("sans-serif", 30))
                .x_label_area_size(40)
                .y_label_area_size(50)
                .build_cartesian_2d(0..N_DATA_POINTS as u32, -5f32..100f32)?;

            cc.configure_mesh()
                .x_label_formatter(&|x| format!("{}", -(LENGTH as f32) + (*x as f32 / FPS as f32)))
                .y_label_formatter(&|y| format!("{:.1}", *y))
                .x_labels(15)
                .y_labels(5)
                .x_desc("Seconds")
                .y_desc("Flexion angle")
                .axis_desc_style(("sans-serif", 15))
                .draw()?;

            self.epoch += 1;
            Ok(())
        });
    }
}
