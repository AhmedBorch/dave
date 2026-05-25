"""Blender headless script: load a DAE, decimate every mesh, export as DAE.

Run with:
    blender --background --python decimate.py -- INPUT.dae OUTPUT.dae RATIO

RATIO is the fraction of triangles to keep (e.g. 0.05 -> 5%).
"""
import sys

import bpy


def parse_args():
    if '--' not in sys.argv:
        raise SystemExit('Usage: blender -b -P decimate.py -- IN.dae OUT.dae RATIO')
    extra = sys.argv[sys.argv.index('--') + 1:]
    if len(extra) != 3:
        raise SystemExit('Need IN.dae OUT.dae RATIO')
    return extra[0], extra[1], float(extra[2])


def main():
    in_path, out_path, ratio = parse_args()

    # Wipe default scene
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)

    print(f'[decimate] importing {in_path}', flush=True)
    bpy.ops.wm.collada_import(filepath=in_path)

    meshes = [o for o in bpy.context.scene.objects if o.type == 'MESH']
    print(f'[decimate] {len(meshes)} mesh objects loaded', flush=True)

    total_before = 0
    total_after = 0
    for obj in meshes:
        n_before = len(obj.data.polygons)
        total_before += n_before

        mod = obj.modifiers.new(name='dec', type='DECIMATE')
        mod.decimate_type = 'COLLAPSE'
        mod.ratio = ratio

        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.modifier_apply(modifier=mod.name)

        n_after = len(obj.data.polygons)
        total_after += n_after
        print(f'[decimate]   {obj.name}: {n_before} -> {n_after}', flush=True)

    print(f'[decimate] total triangles: {total_before} -> {total_after}', flush=True)

    bpy.ops.object.select_all(action='SELECT')
    print(f'[decimate] exporting {out_path}', flush=True)
    bpy.ops.wm.collada_export(filepath=out_path, apply_modifiers=True,
                               selected=True, include_children=True)
    print('[decimate] done', flush=True)


if __name__ == '__main__':
    main()
