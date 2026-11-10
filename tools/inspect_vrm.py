#!/usr/bin/env python3
"""Inspect a local VRM without uploading or changing the asset."""
import argparse
import json
import struct
from pathlib import Path


def inspect(path):
    with path.open('rb') as f:
        magic, version, length = struct.unpack('<4sII', f.read(12))
        if magic != b'glTF' or version != 2 or length != path.stat().st_size:
            raise ValueError('Not a valid GLB 2 file')
        size, kind = struct.unpack('<II', f.read(8))
        if kind != 0x4E4F534A or size > length - 20:
            raise ValueError('Missing JSON chunk')
        data = json.loads(f.read(size))
        bin_size, bin_kind = struct.unpack('<II', f.read(8))
        if bin_kind != 0x004E4942:
            raise ValueError('Missing binary chunk')
        bin_start = f.tell()
        for image in data.get('images', []):
            if 'bufferView' not in image:
                continue
            view = data['bufferViews'][image['bufferView']]
            f.seek(bin_start + view.get('byteOffset', 0))
            header = f.read(24)
            if header[:8] == b'\x89PNG\r\n\x1a\n':
                image['_dimensions'] = list(struct.unpack('>II', header[16:24]))
    ext = data.get('extensions', {})
    vrm = ext.get('VRMC_vrm', ext.get('VRM', {}))
    expressions = vrm.get('expressions', {})
    return {
        'bytes': length, 'version': vrm.get('specVersion', vrm.get('exporterVersion')),
        'format': 'VRM 1' if 'VRMC_vrm' in ext else 'VRM 0', 'meta': vrm.get('meta', {}),
        'expressions': list(expressions.get('preset', {})) + list(expressions.get('custom', {}))
        if expressions else [x.get('presetName') or x.get('name') for x in vrm.get('blendShapeMaster', {}).get('blendShapeGroups', [])],
        'springs': len(ext.get('VRMC_springBone', {}).get('springs', vrm.get('secondaryAnimation', {}).get('boneGroups', []))),
        'materials': len(data.get('materials', [])), 'meshes': len(data.get('meshes', [])),
        'textures': len(data.get('textures', [])),
        'images': [{'name': x.get('name'), 'mime': x.get('mimeType'), 'dimensions': x.get('_dimensions'),
                    'bytes': data['bufferViews'][x['bufferView']]['byteLength'] if 'bufferView' in x else None}
                   for x in data.get('images', [])],
    }


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('path', type=Path)
    print(json.dumps(inspect(ap.parse_args().path), ensure_ascii=False, indent=2))
