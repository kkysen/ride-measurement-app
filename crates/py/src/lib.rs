//! Python bindings for the session reader. Arrays are returned as raw little-endian bytes
//! for the Python side to wrap with `numpy.frombuffer`, which keeps every bit and avoids a numpy build dependency.

use pyo3::exceptions::{PyIOError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyBytes, PyDict};
use ride_session::read::Session;

/// Reads one stream: returns `(timestamps_ns: bytes of i64, values: bytes of f32, values_per_sample, accuracy: bytes of i32, truncated)`.
#[pyfunction]
fn read_stream<'py>(
    py: Python<'py>,
    session_dir: &str,
    stream: &str,
) -> PyResult<Bound<'py, PyDict>> {
    let session = Session::open(session_dir).map_err(|e| PyIOError::new_err(format!("{e:#}")))?;
    let contents = session
        .read_stream(stream)
        .map_err(|e| PyIOError::new_err(format!("{e:#}")))?;
    let mut timestamps = Vec::new();
    let mut values = Vec::new();
    let mut accuracy = Vec::new();
    // Chunks of a stream normally all have the same width; if a sensor ever changed it, say so
    // rather than silently reshaping values into the wrong columns.
    let values_per_sample = contents.chunks.first().map_or(0, |c| c.values_per_sample);
    if let Some(c) = contents
        .chunks
        .iter()
        .find(|c| c.values_per_sample != values_per_sample)
    {
        return Err(PyValueError::new_err(format!(
            "{stream}: values_per_sample changes from {values_per_sample} to {} within the stream",
            c.values_per_sample
        )));
    }
    for chunk in &contents.chunks {
        timestamps.extend(chunk.timestamps_ns().iter().flat_map(|t| t.to_le_bytes()));
        values.extend(chunk.value_bits.iter().flat_map(|b| b.to_le_bytes()));
        accuracy.extend(chunk.accuracy.iter().flat_map(|a| a.to_le_bytes()));
    }
    let out = PyDict::new(py);
    out.set_item("timestamps_ns", PyBytes::new(py, &timestamps))?;
    out.set_item("values", PyBytes::new(py, &values))?;
    out.set_item("values_per_sample", values_per_sample)?;
    out.set_item("accuracy", PyBytes::new(py, &accuracy))?;
    out.set_item("truncated", contents.truncated)?;
    out.set_item("damaged_ranges", contents.damage.len())?;
    Ok(out)
}

/// Stream names in the session.
#[pyfunction]
fn stream_names(session_dir: &str) -> PyResult<Vec<String>> {
    let session = Session::open(session_dir).map_err(|e| PyIOError::new_err(format!("{e:#}")))?;
    Ok(session.info.streams.into_iter().map(|s| s.name).collect())
}

#[pymodule]
fn _native(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(read_stream, m)?)?;
    m.add_function(wrap_pyfunction!(stream_names, m)?)?;
    Ok(())
}
