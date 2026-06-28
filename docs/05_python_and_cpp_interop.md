# Python ↔ C++ interop — "can the Python files use the C++ `.so`?"

Short answer: **you don't need to, and by default you shouldn't.** Python and C++
interoperate through the **shared `/dev/shm` byte contract**, not by linking the
library. But you *can* call the C++ classes from Python if you want — here are all
three options, with their trade-offs.

---

## Option 1 (recommended) — speak the same /dev/shm contract
This is what the Python examples (`write_py.py`, `read_py.py`, `network_to_shm.py`)
do. The header layout + seqlock are defined identically in:
- C++: [`shm_contract.hpp`](../src/shm_bridge_cpp/include/shm_bridge_cpp/shm_contract.hpp)
- Python: [`shm_contract.py`](../src/shm_bridge_python/shm_bridge_python/shm_contract.py)

Because the on-disk bytes are identical, a **C++ writer is read by a Python reader
and vice-versa, with zero glue code**:

| writer | reader | works? |
|---|---|---|
| C++ `Writer` (`write_cpp`, `ros2_to_shm`) | Python `read_py.py` | ✅ |
| Python `write_py.py` | C++ `Reader` (`read_cpp`, `shm_to_ros2`) | ✅ |
| C++ → C++ | — | ✅ |
| Python → Python | — | ✅ |

**Pros:** no compilation of bindings, no ABI coupling, no extra copies (Python uses
`mmap` + `numpy.frombuffer`, which is a zero-copy view of the buffer). Same speed
class as the C++ path for the read itself.
**Cons:** the seqlock/futex logic is reimplemented in Python (it's ~20 lines and
already written for you). Python doesn't get the C++ `wait_and_read` futex blocking
out of the box — `read_py.py` polls the seq with a 1 ms sleep. For most rates that's
fine; if you need true 0% CPU blocking in Python, use Option 2 or add a futex via
`ctypes`.

**Try it (cross-language, proves the contract):**
```bash
./install_lib.sh                                  # so read_cpp/write_cpp build
g++ -std=c++17 examples/cpp/read_cpp.cpp -o /tmp/read_cpp -lshm_bridge_cpp -lpthread
python3 examples/python/write_py.py demo &        # Python writer
/tmp/read_cpp                                      # C++ reader sees Python's frames
```

---

## Option 2 — call the C++ `.so` from Python via pybind11
If you specifically want Python to drive the actual `shm_bridge::Writer` /
`shm_bridge::Reader` C++ classes (e.g. to reuse the futex `wait_and_read`), wrap
them with **pybind11**. The `.so` is a normal shared library, so this is
straightforward:

```cpp
// pybind_shm.cpp  ->  build into shm_bridge_py.*.so
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <shm_bridge_cpp/shm_bridge.hpp>
namespace py = pybind11;

PYBIND11_MODULE(shm_bridge_py, m) {
    py::enum_<shm_bridge::DType>(m, "DType")
        .value("U8", shm_bridge::DType::U8).value("F32", shm_bridge::DType::F32);  // etc.
    py::class_<shm_bridge::Frame>(m, "Frame")
        .def_readonly("seq", &shm_bridge::Frame::seq)
        .def_readonly("width", &shm_bridge::Frame::width)
        .def_readonly("height", &shm_bridge::Frame::height)
        .def_property_readonly("data", [](const shm_bridge::Frame& f) {
            return py::bytes(reinterpret_cast<const char*>(f.data.data()), f.data.size());
        });
    py::class_<shm_bridge::Reader>(m, "Reader")
        .def(py::init<const std::string&>())
        .def("wait_and_read", [](shm_bridge::Reader& r, uint64_t to) {
            shm_bridge::Frame f; bool ok = r.wait_and_read(f, to);
            return ok ? py::object(py::cast(f)) : py::none();
        }, py::arg("timeout_ns") = 0);
    py::class_<shm_bridge::Writer>(m, "Writer")
        .def(py::init<const std::string&, size_t>())
        .def("write_flat", [](shm_bridge::Writer& w, py::buffer b, uint32_t wd,
                              uint32_t ht, uint32_t ch, shm_bridge::DType dt,
                              const std::string& tn) {
            py::buffer_info bi = b.request();
            return w.write_flat(bi.ptr, bi.size * bi.itemsize, wd, ht, ch, dt, tn);
        });
}
```
Build:
```bash
g++ -O3 -std=c++17 -shared -fPIC pybind_shm.cpp \
    $(python3 -m pybind11 --includes) \
    -I$PREFIX/include -L$PREFIX/lib -lshm_bridge_cpp \
    -o shm_bridge_py$(python3-config --extension-suffix)
```
Use:
```python
import shm_bridge_py
r = shm_bridge_py.Reader("demo")
f = r.wait_and_read(0)        # real C++ futex wait, 0% CPU
print(f.seq, f.width, len(f.data))
```
**Pros:** Python gets the exact C++ behavior incl. futex blocking; one copy into
`py::bytes`. **Cons:** you compile and ship a binding `.so` matched to your Python
version; the binding `.so` depends on `libshm_bridge_cpp.so` at runtime
(`LD_LIBRARY_PATH`).

---

## Option 3 — `ctypes` (no compile, but you must wrap a C API)
`ctypes` can load `libshm_bridge_cpp.so` directly **only if there is an `extern "C"`
API** to call. The current library exposes C++ classes (name-mangled), which
`ctypes` can't call cleanly. If you want this route, add a thin `extern "C"` shim
(e.g. `shm_writer_create`, `shm_write_flat`, `shm_reader_create`,
`shm_wait_and_read`) to the library and load it with `ctypes.CDLL`. More fragile
than pybind11; only worth it if you must avoid a compile step.

---

## Recommendation
- **Default: Option 1.** The contract is the API. It's the simplest, fastest-to-set-up,
  and already proven by the examples; cross-language reads/writes "just work."
- **Use Option 2** only when you need the C++ futex `wait_and_read` (true 0% CPU
  blocking) from Python at high rates.
- **Avoid Option 3** unless you specifically need `ctypes` and are willing to add a
  C shim.

> Bottom line to the literal question: *the Python files do NOT currently use the
> C++ `.so` — they use the shared memory the `.so` writes.* That is intentional and
> is the recommended interop. If you want them to load the `.so`, pybind11 (Option
> 2) is the clean path.
