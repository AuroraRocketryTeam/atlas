---
name: Feature Request / Engineering Task
about: Proposal for a new feature, refactoring, or technical task
title: "[FEATURE/TASK] Short description here"
labels: enhancement, discussion
assignees: ''
---

## 📋 Summary
A concise description of the change. Example: "Implement a native unit test suite for the Kalman Filter to validate apogee detection logic."

## 🚀 Motivation & Value
Explain why this change is valuable.
* **User Value:** ...
* **Technical Value:** (e.g., Improves code coverage, decouples logic from hardware).

## ⚙️ Technical Details / Scope
* **Target Module/Class:** `ClassName` (e.g., `lib/kalman/src/KalmanFilter1D.cpp`)
* **Proposed Changes:**
    1.  Create ...
    2.  Refactor ...
* **Constraints:** (e.g., Must run on `native` environment, no heap allocation allowed).
<!---
## 🧪 Test Plan (Mandatory)
Describe the tests that need to be written or updated.
* [ ] Create unit tests covering happy paths.
* [ ] Create unit tests for edge cases (e.g., null pointers, zero values).
* [ ] Ensure `pio test -e native` passes.
--->
## ✅ Definition of Done (DoD)
* [ ] Implementation is complete and follows OOP/SOLID principles.
* [ ] Code is documented (Doxygen style).
<!--- * [ ] Unit tests are implemented and passing. --->
* [ ] No new compiler warnings.
* [ ] KISS principle respected (no over-engineering).

## 🔗 References
* Related Issue: #...
* Docs: ...
