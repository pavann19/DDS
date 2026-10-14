"""Read-only timestamped telemetry capture from the actual running sandbox."""
import argparse
import asyncio
import json
import time
from pathlib import Path
import websockets

# Initial real-road geometry can exceed websockets' 1 MiB default. Keep a
# finite transport bound while accommodating long OSRM routes.
MAX_MESSAGE_BYTES = 16 * 1024 * 1024


def safety_failure(shield):
    """Recorded overlap is a failed drive even if road containment holds."""
    shield = shield or {}
    if shield.get('body_contained') is False or shield.get('swept_body_contained') is False:
        return 'road_boundary_failure'
    if shield.get('overlap_detected') is True:
        return 'occupied_space_overlap'
    return None


async def capture(seconds, output, until_parked=False, url='ws://127.0.0.1:8001/ws/telemetry'):
    rows=[]
    output.parent.mkdir(parents=True,exist_ok=True)
    journal=output.with_suffix('.jsonl').open('w',encoding='utf-8',buffering=1)
    run_identity=None
    stop_reason='duration_elapsed'
    try:
      async with websockets.connect(url, max_size=MAX_MESSAGE_BYTES) as socket:
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            message=json.loads(await asyncio.wait_for(socket.recv(),5))
            if message.get('type')!='state':continue
            identity=(message['simulation_id'],message['epoch'])
            if run_identity is not None and identity!=run_identity:
                stop_reason='simulation_or_epoch_changed'
                break
            run_identity=identity
            ego=message['channels']['pose']['ego']
            rows.append({'wall_s':time.time(),'tick':message['tick'],'sim_s':message['simulation_time_s'],
                         'simulation_id':message['simulation_id'],'epoch':message['epoch'],
                         'ego':ego,'shield':message['channels']['semantic'].get('safety_shield'),
                         'planner':message['channels']['semantic'].get('planner'),
                         'scenario':message['channels']['semantic'].get('scenario')})
            # Keep every received sample if the process/window is interrupted.
            journal.write(json.dumps(rows[-1],separators=(',',':'))+'\n')
            shield=rows[-1].get('shield') or {}
            failure = safety_failure(shield)
            if until_parked and failure:
                stop_reason=failure
                break
            if until_parked and (ego.get('parking') or {}).get('parked'):
                stop_reason='parked'
                break
    finally:
      journal.close()
      output.write_text(json.dumps(rows,indent=2),encoding='utf-8')
    ready=sum(bool(r['ego'].get('sensor_health',{}).get('ready')) for r in rows)
    failed=sum((r.get('shield') or {}).get('body_contained') is False or (r.get('shield') or {}).get('swept_body_contained') is False for r in rows)
    overlaps=sum((r.get('shield') or {}).get('overlap_detected') is True for r in rows)
    parked=bool(rows and (rows[-1]['ego'].get('parking') or {}).get('parked'))
    print(json.dumps({'samples':len(rows),'sensor_ready_samples':ready,'body_outside_samples':failed,'overlap_samples':overlaps,
                      'station_first':-rows[0]['ego']['pose']['z'] if rows else None,
                      'station_last':-rows[-1]['ego']['pose']['z'] if rows else None,
                      'parked':parked,'goal_timeout':until_parked and not parked and stop_reason=='duration_elapsed','stop_reason':stop_reason}))
    return bool(rows) and stop_reason!='simulation_or_epoch_changed' and not failed and not overlaps and (parked or not until_parked)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--seconds',type=float,default=15)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--until-parked',action='store_true')
    parser.add_argument('--url',default='ws://127.0.0.1:8001/ws/telemetry')
    args=parser.parse_args()
    if not asyncio.run(capture(args.seconds,args.output,args.until_parked,args.url)):
        raise SystemExit(2)
