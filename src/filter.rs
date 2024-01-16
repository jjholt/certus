use std::{f64::consts::PI, collections::VecDeque};

pub struct Butterworth {
    coefficients: Vec<f64>,
    input: VecDeque<f64>,
    output: VecDeque<f64>,
    order: usize,
}

impl Butterworth {
    pub fn new(order: usize, cutoff_frequency: f64, sampling_frequency: f64) -> Self {
        let omega_c_d = 2.0 * PI * cutoff_frequency / sampling_frequency;
        let coefficients = Self::coefficients(order, omega_c_d);
        let input = VecDeque::from(vec![0.0; order]);
        let output = VecDeque::from(vec![0.0; order]);
        Self { coefficients, input, output, order }
    }
    pub fn filter(&mut self, input: f64) -> f64 {
        self.input.push_front(input);
        self.input.pop_back();

        let output: f64 = (0..self.order).map(|i| self.coefficients[i] * (self.input[i] - self.output[i])).sum();
        self.output.push_front(output);
        self.output.pop_back();
        output
    }
    fn coefficients(order: usize, omega_c_d: f64) -> Vec<f64> {
        let t = 0.5 * omega_c_d;

        (0..order).map(|k| {
            let f: f64 = (2*k+1) as f64 *PI/(2*order) as f64;
            let a = (2.0*t*f.sin()).powi(2);
            let b = 4.0*t.powi(2) + 1.0;
            a/b
        }).collect()
    }
}
