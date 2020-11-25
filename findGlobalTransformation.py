#!/usr/bin/env python
"""
Based on a file from evo library 

test/demo for trajectory alignment functions
author: Michael Grupp

This file is part of evo (github.com/MichaelGrupp/evo).

evo is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

evo is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with evo.  If not, see <http://www.gnu.org/licenses/>.
"""

import logging
import sys
import argparse

import evo.core.lie_algebra as lie
from evo.core import trajectory
from evo.tools import plot, file_interface, log

import numpy as np
import matplotlib.pyplot as plt

def findTransform(poses, sfm_output):
    logger = logging.getLogger("evo")
    log.configure_logging(verbose=True)

    traj_ref = file_interface.read_tum_trajectory_file(poses)
    traj_est = file_interface.read_tum_trajectory_file(sfm_output)

    logger.info("\nUmeyama alignment with scaling")
    traj_est_aligned_scaled = trajectory.align_trajectory(traj_est, traj_ref,
                                                        correct_scale=True)


    fig = plt.figure(figsize=(16,16))
    plot_mode = plot.PlotMode.xy

    ax = plot.prepare_axis(fig, plot_mode)
    plot.traj(ax, plot_mode, traj_ref, '-', 'gray')
    plot.traj(ax, plot_mode, traj_est_aligned_scaled, '-', 'blue')
    fig.axes.append(ax)
    plt.title('$\mathrm{Sim}(3)$ alignment')

    fig.tight_layout()
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('sfm_output', type=str, help="YAML file containing the parameters")
    parser.add_argument('poses', type=str, help="Folder containing the output files")

    resParser = parser.parse_args()

    findTransform(resParser.poses, resParser.sfm_output)