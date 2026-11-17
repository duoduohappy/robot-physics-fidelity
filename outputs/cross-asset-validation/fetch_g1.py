import concurrent.futures
import hashlib
import json
import pathlib
import urllib.request
import traceback
import time
import sys

ROOT=pathlib.Path(__file__).resolve().parents[2]
OUT=ROOT/'outputs/cross-asset-validation'
ASSET=ROOT/'work/g1-original'
COMMIT='71f066ad0be9cd271f7ed58c030243ef157af9f4'

def fetch():
    manifest=json.loads((pathlib.Path(__file__).resolve().parent/'mesh-manifest.json').read_text(encoding='utf-8'))[1]
    entries=manifest['meshes']+[{'path':'unitree_g1/'+n} for n in ['scene_mjx.xml','g1_mjx.xml','LICENSE']]
    def get(row):
        relative=pathlib.PurePosixPath(row['path']).relative_to('unitree_g1')
        path=ASSET.joinpath(*relative.parts)
        if path.exists():
            data=path.read_bytes()
        else:
            data=urllib.request.urlopen('https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/'+COMMIT+'/'+row['path'],timeout=60).read()
            path.parent.mkdir(parents=True,exist_ok=True)
            path.write_bytes(data)
        gitsha=hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
        if 'git_blob_sha1' in row and (gitsha!=row['git_blob_sha1'] or len(data)!=row['bytes']):
            raise ValueError('Integrity failure '+row['path'])
        return {'path':row['path'],'bytes':len(data),'git_blob_sha1':gitsha,'sha256':hashlib.sha256(data).hexdigest()}
    rows=list(concurrent.futures.ThreadPoolExecutor(4).map(get,entries))
    OUT.mkdir(exist_ok=True)
    (OUT/'g1-download-manifest.json').write_text(json.dumps({'commit':COMMIT,'files':rows},indent=2),encoding='utf-8')
    print('FETCH_OK',len(rows),sum(r['bytes'] for r in rows),flush=True)

if __name__=='__main__': fetch()
