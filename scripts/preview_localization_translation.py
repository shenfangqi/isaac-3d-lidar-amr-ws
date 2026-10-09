#!/usr/bin/env python3
"""Read-only translation feasibility from recorded stationary /scan frames.

No ROS nodes, publishers, profile approval or physical motion. The requested
stopping extension is an analysis assumption, never a calibration result.
"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
from types import SimpleNamespace as NS

from diagnose_localization_bag import read_bag, state_intervals, select_frames
from isaac_3d_lidar_bringup.localization_contracts import FrameRole
from isaac_3d_lidar_bringup.localization_rotation_policy import evidence_from_scan
from isaac_3d_lidar_bringup.localization_translation_policy import preview_translation, choose_translation

FOOTPRINT = ((.155,.133),(.155,-.133),(-.130,-.133),(-.130,.133))


def analyze(bag, scan_topic, stopping_extension):
    scans, odom, statuses, static, counts, types, session = read_bag(bag,scan_topic)
    output = dict(bag=str(bag), session=session, scan_topic=scan_topic,
                  motion_authorized=False, stopping_extension_assumption_m=stopping_extension,
                  footprint=FOOTPRINT, views=[])
    used=set()
    for interval in state_intervals(statuses):
        if interval['state'] != 'COLLECT_STATIC':
            continue
        view=len(output['views'])
        record=dict(view=view)
        output['views'].append(record)
        try:
            frames,_=select_frames(scans,odom,static,interval['start_ns'],interval['end_ns'],
                                    session,view,FrameRole.TRAIN,1,used,view)
            f=frames[0]
            scan=NS(**asdict(f.scan))
            scan.header=NS(frame_id=scan.frame_id,stamp=NS(
                sec=scan.stamp_ns//10**9,nanosec=scan.stamp_ns%10**9))
            pose=f.T_odom_base
            evidence=evidence_from_scan(scan,pose.compose(f.T_base_scan),
                                        (pose.x,pose.y),half_extent_m=2.,resolution=.025)
            previews=tuple(preview_translation(evidence,FOOTPRINT,pose,d,
                            stop_extension_m=stopping_extension,now_ns=scan.stamp_ns)
                           for d in (.2,.4,.6))
            selected=choose_translation(previews)
            initial=preview_translation(evidence,FOOTPRINT,pose,1e-6,
                                        stop_extension_m=0.,now_ns=scan.stamp_ns)
            record.update(stamp_ns=scan.stamp_ns,
                          initial_clearance=asdict(initial),
                          previews=[asdict(p) for p in previews],
                          selected_distance_m=selected.distance_m if selected else None)
        except (ValueError, RuntimeError) as error:
            record['error']=str(error)
    output['evaluated_views']=sum('previews' in v for v in output['views'])
    output['feasible_views']=sum(v.get('selected_distance_m') is not None for v in output['views'])
    return output


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag',type=Path)
    parser.add_argument('--scan-topic',default='/scan')
    parser.add_argument('--stopping-extension-m',type=float,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=analyze(args.bag,args.scan_topic,args.stopping_extension_m)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:result[k] for k in ('evaluated_views','feasible_views','motion_authorized')}))


if __name__ == '__main__':
    main()
