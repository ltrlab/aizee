# Source for the simulation environment on coolserver (aizee-sim only). TREE defaults to the 01e2198 tree.
export AIZEE_SIM=${AIZEE_SIM:-$HOME/aizee-sim}
export TREE=${TREE:-$AIZEE_SIM/tree-01e2198}
export PYTHONPATH=$TREE/src
export ROBOT_CONTROL_HARNESS=$AIZEE_SIM/bin/control_harness
export ROBOT_COLLISION_LIBRARY=$AIZEE_SIM/bin/librobot_collision.so
# MuJoCo renders on GPU 0 through EGL (EGL device 0 is nvidia-smi GPU 0 on this box, checked 2026-10-18).
export MUJOCO_GL=egl
export MUJOCO_EGL_DEVICE_ID=${MUJOCO_EGL_DEVICE_ID:-0}
export PYOPENGL_PLATFORM=egl
# One BLAS/OpenMP thread per process: the box is shared and each worker already runs several processes.
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1
# Temporary files stay inside aizee-sim.
export TMPDIR=$AIZEE_SIM/tmp
