#!/usr/bin/env python3

import argparse

if __name__ == '__main__':
    # Init parser:
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input', help='Footstep plan as .tsv file.')
    parser.add_argument('-o', '--output', help='Footstep plan as .txt file.')

    # Parse arguments:
    args = parser.parse_args()
    footstep_plan_tsv_file_path = args.input
    footstep_plan_txt_file_path = args.output

    # Read .tsv and convert it to .txt:
    with open(footstep_plan_txt_file_path, 'w') as footstep_plan_txt_file:
        footstep_plan_txt_file.write('map\n')
        with open(footstep_plan_tsv_file_path, 'r') as footstep_plan_tsv_file:
            footstep_plan_tsv_file.readline()
            for l in footstep_plan_tsv_file.readlines():
                idx, q_supp, q_swg, support_foot, traj_params, h_max = l.rstrip().split('\t')
                qL = q_supp if support_foot == 'Left' else q_swg
                qR = q_supp if support_foot == 'Right' else q_swg
                v = []
                v.extend([float(x) for x in qL.strip('[').rstrip(']').split(',')])
                v.extend([float(x) for x in qR.strip('[').rstrip(']').split(',')])
                v.append(support_foot.upper())
                v.append(0.0 if h_max == 'None' else float(h_max))
                footstep_plan_txt_file.write(','.join([str(x) for x in v]) + '\n')
