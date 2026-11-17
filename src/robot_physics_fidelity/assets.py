"""Explicit, hash-checked fetch of the two fixed public demo assets."""
from pathlib import Path
from importlib.resources import files
import concurrent.futures
import hashlib
import json
import urllib.request

def fetch(asset,directory):
    manifest=json.loads(files('robot_physics_fidelity').joinpath('assets-lock.json').read_text(encoding='utf-8'))[asset]
    root=Path(directory).resolve()/asset
    def one(row):
        relative=Path(*row['path'].split('/')[1:]); target=(root/relative).resolve()
        if root not in target.parents: raise ValueError('Invalid asset manifest path')
        if target.exists(): data=target.read_bytes()
        else: data=urllib.request.urlopen('https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/'+manifest['commit']+'/'+row['path'],timeout=60).read()
        digest=hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
        if len(data)!=row['bytes'] or digest!=row['git_blob_sha1']: raise ValueError('Asset checksum mismatch: '+row['path'])
        if not target.exists(): target.parent.mkdir(parents=True,exist_ok=True); target.write_bytes(data)
        return len(data)
    size=sum(concurrent.futures.ThreadPoolExecutor(4).map(one,manifest['files']))
    return {'status':'FETCHED','files':len(manifest['files']),'bytes':size,'scene':str(root/manifest['scene']),'commit':manifest['commit']}
