# Evaluation tools of the estimation module: synthetic drives with ground truth, consistency and
# map metrics, and the batch least-squares reference. Tests and benchmarks link this; the car's
# software does not.
apex_module(estimation_eval
    SOURCES
        consistency.cpp
        map_metrics.cpp
        synthetic.cpp
    PUBLIC_DEPS apex::estimation)
