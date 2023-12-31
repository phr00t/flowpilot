ARCHNAME=$(uname -m)
SCRIPT=$(realpath "$0")
FLOWPILOT_DIR=$(dirname "$SCRIPT")

ENV_FILE=$FLOWPILOT_DIR/.env
rm -f $ENV_FILE

echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:\"${FLOWPILOT_DIR}/libs/linux/${ARCHNAME}\"" >> $ENV_FILE
echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:\"${FLOWPILOT_DIR}/third_party/acados/${ARCHNAME}/lib\"" >> $ENV_FILE
echo "export PYTHONPATH=\$PYTHONPATH:\"${FLOWPILOT_DIR}/third_party/acados\"" >> $ENV_FILE
echo "export ACADOS_SOURCE_DIR=\"${FLOWPILOT_DIR}/third_party/acados/include/acados\"" >> $ENV_FILE
echo "export ACADOS_PYTHON_INTERFACE_PATH=\"${FLOWPILOT_DIR}/third_party/acados/acados_template\"" >> $ENV_FILE
echo "export TERA_PATH=\"${FLOWPILOT_DIR}/third_party/acados/${ARCHNAME}/t_renderer\"" >> $ENV_FILE

