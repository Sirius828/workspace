"""
Namespace package for *yolo_detector*.
This __init__.py turns *yolo_detector* into a PEP‑420 namespace package so
that the runtime node code inside this directory and the rosidl‑generated
message sub‑package installed to
<workspace>/install/yolo_detector/.../site-packages/yolo_detector
can coexist.
Only extend the search path here; do **not** import anything else, otherwise
sub‑modules such as ``yolo_detector.msg`` might get shadowed.
"""
from pkgutil import extend_path

# Merge all physical directories named "yolo_detector" into a single
# logical package (node code + rosidl messages).
__path__ = extend_path(__path__, __name__)

