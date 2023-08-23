## Original openpilot log capnp files

Flowpilot's and openpilot's capnp structs are not compatible with each other. Reading openpilot's rlogs with flowpilot's capnp strucs and vice-versa will lead to undefined behaviour. To use openpilot capnp structs for specific usecases by default, set `export OP_CAPNP="1"` before importing log from cereal.