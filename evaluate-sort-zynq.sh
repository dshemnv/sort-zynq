#!/bin/bash

if [ ! -d "extern/trackeval" ]; then
    echo "Trackeval does not exist, please recurse the submodules"
    exit 1
fi

if [[ ! -d ".venv" ]]; then
    if [[! "$(python --version)" =~ "3.7" ]]; then
        echo "Python virtualenv not found or python version is not 3.7"
        echo "Your have ""$(python --version)"
        echo "Then create a venv: python -m venv .venv"
        exit 1
    fi
else
    source .venv/bin/activate
fi

if [ ! -d "extern/trackeval/data/gt/mot_challenge" ]; then
    echo "Please download the ground truth for MOT Challenge"
    echo "More details on TrackEval github page"
    exit 1
fi


python extern/trackeval/scripts/run_mot_challenge.py \
--USE_PARALLEL True \
--PLOT_CURVES True \
--METRICS HOTA CLEAR Identity \
--GT_FOLDER extern/trackeval/data/gt/mot_challenge \
--TRACKERS_FOLDER bench_results/sort/data \
--TRACKER_SUB_FOLDER "" \
--BENCHMARK MOT15 \
--SPLIT_TO_EVAL train

deactivate
