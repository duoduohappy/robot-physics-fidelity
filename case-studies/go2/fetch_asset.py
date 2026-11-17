"""Fetch only fixed Go2 files; preserve upstream bytes and license."""
import argparse
import concurrent.futures
import hashlib
import json
from pathlib import Path
import time
import urllib.request


def fetch(directory):
    manifest=json.loads(Path(__file__).with_name('asset-lock.json').read_text(encoding='utf-8'))
    root=Path(directory).resolve()
    expected=sum(r['bytes'] for r in manifest['files'])
    if expected>manifest['maximum_download_bytes']:
        raise ValueError('Manifest exceeds declared download bound')
    def one(row):
        parts=Path(row['path']).parts
        if parts[0]!='unitree_go2':raise ValueError('Unexpected asset prefix')
        target=(root/Path(*parts[1:])).resolve()
        if root not in target.parents:raise ValueError('Path outside asset directory')
        if target.exists():data=target.read_bytes()
        else:
            url='https://raw.githubusercontent.com/'+manifest['repository']+'/'+manifest['commit']+'/'+row['path']
            for attempt in range(3):
                try:
                    with urllib.request.urlopen(url,timeout=45) as response:
                        data=response.read(row['bytes']+1)
                    break
                except Exception:
                    if attempt==2:raise
                    time.sleep(1)
        git=hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
        if len(data)!=row['bytes'] or git!=row['git_blob_sha1']:
            raise ValueError('Checksum mismatch: '+row['path'])
        if not target.exists():
            target.parent.mkdir(parents=True,exist_ok=True)
            with target.open('xb') as file:file.write(data)
        return {**row,'sha256':hashlib.sha256(data).hexdigest()}
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        files=list(pool.map(one,manifest['files']))
    result={'status':'FETCHED_AND_VERIFIED','commit':manifest['commit'],'files':files,'bytes':expected,'source_modified':False}
    root.mkdir(parents=True,exist_ok=True)
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('--directory',required=True);p.add_argument('--report',required=True);a=p.parse_args()
    result=fetch(a.directory)
    report=Path(a.report);report.parent.mkdir(parents=True,exist_ok=True)
    report.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='files'},indent=2))


if __name__=='__main__':main()
