#!/bin/bash

if [ ! -d "extern/trackeval" ]; then
	echo "Trackeval does not exist, please recurse the submodules"
	exit 1
fi

if [[ ! -d ".venv" || ! "$(python --version)" =~ "3.7" ]]; then
	echo "Python virtualenv not found or python version is not 3.7"
	echo "Your have ""$(python --version)"
	echo "Then create a venv: python -m venv .venv"
	exit 1
fi

if [ ! -d "extern/trackeval/data/gt/mot_challenge" ]; then
	echo "Please download the ground truth for MOT Challenge"
	echo "More details on TrackEval github page"
	exit 1
fi

source .venv/bin/activate

python extern/trackeval/scripts/run_mot_challenge.py \
	--USE_PARALLEL True \
	--PLOT_CURVES True \
	--METRICS Hota CLEAR \
	--GT_FOLDER extern/trackeval/data/gt/mot_challenge \
	--TRACKERS_FOLDER bench_results/sort/data \
	--TRACKER_SUB_FOLDER "" \
	--BENCHMARK MOT15 \
	--SPLIT_TO_EVAL train