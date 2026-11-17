"""Experimental, bounded USD contact-pair bridge; not an official USD schema.

Uses the official converter and Newton APIs without modifying installed code.
Supports named primitive geoms, rigid bodies and CPU MuJoCo consumption only.
"""
from pathlib import Path
import math
import mujoco
import mujoco_usd_converter
from pxr import Usd, UsdPhysics, Sdf

PREFIX = 'contactBridge:'
FIELDS = {'condim': ('pair_dim', Sdf.ValueTypeNames.Int, None),
          'friction': ('pair_friction', Sdf.ValueTypeNames.DoubleArray, 5),
          'solref': ('pair_solref', Sdf.ValueTypeNames.DoubleArray, 2),
          'solreffriction': ('pair_solreffriction', Sdf.ValueTypeNames.DoubleArray, 2),
          'solimp': ('pair_solimp', Sdf.ValueTypeNames.DoubleArray, 5),
          'margin': ('pair_margin', Sdf.ValueTypeNames.Double, None),
          'gap': ('pair_gap', Sdf.ValueTypeNames.Double, None)}

def attr(prim, name, typ, value):
    prim.CreateAttribute(PREFIX + name, typ, custom=True).Set(value)

def export_asset(source, output):
    output = Path(output)
    if output.exists():
        raise ValueError('Output must not exist')
    model = mujoco.MjModel.from_xml_path(str(Path(source).resolve()))
    if model.ngeom > 256:
        raise ValueError('Pilot supports at most 256 geoms')
    if model.ntendon or model.neq or model.nu:
        raise ValueError('Pilot excludes tendons, equalities and actuators')
    if any(t != mujoco.mjtJoint.mjJNT_FREE for t in model.jnt_type) or model.nmocap:
        raise ValueError('Pilot qualifies independent free rigid bodies, not articulations or mocap')
    if any(model.body_geomnum[b] != 1 for b in range(1, model.nbody)):
        raise ValueError('Pilot requires one primitive geom per non-world body')
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) for i in range(model.ngeom)]
    if any(not n or not Sdf.Path.IsValidIdentifier(n) for n in names):
        raise ValueError('Pilot requires named USD-valid geom identifiers')
    if any(t not in (mujoco.mjtGeom.mjGEOM_PLANE, mujoco.mjtGeom.mjGEOM_SPHERE,
                     mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_CAPSULE,
                     mujoco.mjtGeom.mjGEOM_CYLINDER) for t in model.geom_type):
        raise ValueError('Only primitive collision geometry is qualified')
    output.mkdir(parents=True)
    asset = mujoco_usd_converter.Converter().convert(str(Path(source).resolve()), str(output / 'official'))
    original = Usd.Stage.Open(asset.path)
    stage = Usd.Stage.Open(original.Flatten())
    geoms = []
    endpoints = set(model.pair_geom1) | set(model.pair_geom2)
    for i, name in enumerate(names):
        candidates = [p for p in stage.Traverse() if p.GetName() == name and p.GetTypeName() in
                      ('Sphere', 'Plane', 'Cube', 'Capsule', 'Cylinder')]
        if len(candidates) != 1:
            raise ValueError('Ambiguous or missing geom path: ' + name)
        p = candidates[0]
        attr(p, 'contype', Sdf.ValueTypeNames.Int64, int(model.geom_contype[i]))
        attr(p, 'conaffinity', Sdf.ValueTypeNames.Int64, int(model.geom_conaffinity[i]))
        if i in endpoints:
            UsdPhysics.CollisionAPI.Apply(p).CreateCollisionEnabledAttr(True)
        geoms.append(p.GetPath())
    root = stage.GetDefaultPrim().GetPath()
    for i in range(model.npair):
        p = stage.DefinePrim(root.AppendChild('ContactBridgePairs').AppendChild('pair_' + str(i)), 'Scope')
        attr(p, 'version', Sdf.ValueTypeNames.Int, 1)
        p.CreateRelationship(PREFIX + 'geom1').SetTargets([geoms[int(model.pair_geom1[i])]])
        p.CreateRelationship(PREFIX + 'geom2').SetTargets([geoms[int(model.pair_geom2[i])]])
        for field, (array, typ, length) in FIELDS.items():
            value = getattr(model, array)[i]
            value = value.tolist() if length else (int(value) if field == 'condim' else float(value))
            attr(p, field, typ, value)
    attr(stage.GetDefaultPrim(), 'formatVersion', Sdf.ValueTypeNames.Int, 1)
    target = output / 'bridged.usda'
    stage.GetRootLayer().Export(str(target))
    return target

def inspect(stage):
    """Validate the entire contract before mutating a Newton builder."""
    shapes, pairs = {}, []
    format_markers = 0
    for p in stage.Traverse():
        fmt = p.GetAttribute(PREFIX + 'formatVersion')
        if fmt:
            if fmt.Get() != 1:
                raise ValueError('Unknown asset contract version')
            format_markers += 1
        if p.IsInstance() or p.IsInstanceProxy():
            raise ValueError('Native USD instancing is not qualified; use ordinary references')
        ct = p.GetAttribute(PREFIX + 'contype')
        if ct:
            ca = p.GetAttribute(PREFIX + 'conaffinity').Get()
            t = ct.Get()
            if not isinstance(t, int) or not isinstance(ca, int):
                raise ValueError('Invalid collision masks')
            shapes[str(p.GetPath())] = (t, ca)
        if p.HasAPI(UsdPhysics.CollisionAPI) and not ct:
            raise ValueError('Unmarked collider outside the supported contract')
        version = p.GetAttribute(PREFIX + 'version')
        if not version:
            continue
        if version.Get() != 1:
            raise ValueError('Unknown pair contract version')
        ends = []
        for field in ('geom1', 'geom2'):
            targets = p.GetRelationship(PREFIX + field).GetTargets()
            endpoint = stage.GetPrimAtPath(targets[0]) if len(targets) == 1 else None
            if not endpoint or not endpoint.HasAPI(UsdPhysics.CollisionAPI):
                raise ValueError('Missing or non-collision pair endpoint')
            ends.append(str(targets[0]))
        if ends[0] == ends[1]:
            raise ValueError('Self pair')
        values = {}
        for field, (_, _, length) in FIELDS.items():
            value = p.GetAttribute(PREFIX + field).Get()
            if length:
                if value is None or len(value) != length:
                    raise ValueError('Invalid pair vector: ' + field)
                value = list(value)
            if value is None or not all(math.isfinite(float(v)) for v in (value if length else [value])):
                raise ValueError('Invalid pair numeric field')
            values[field] = value
        if values['condim'] not in (1, 3, 4, 6) or min(values['friction']) < 0:
            raise ValueError('Invalid contact dimensionality or friction')
        pairs.append((ends, values))
    keys = [tuple(sorted(ends)) for ends, _ in pairs]
    if len(keys) != len(set(keys)):
        raise ValueError('Duplicate pair definition')
    if not format_markers or not shapes or len(shapes) > 256:
        raise ValueError('Unsupported or unmarked stage')
    for ends, _ in pairs:
        if any(e not in shapes for e in ends):
            raise ValueError('Unmarked endpoint')
    return shapes, pairs

def import_asset(builder, filename):
    import newton
    stage = Usd.Stage.Open(str(filename))
    shapes, pairs = inspect(stage)
    if builder.shape_count:
        raise ValueError('Import a composed stage into an empty builder')
    newton.solvers.SolverMuJoCo.register_custom_attributes(builder)
    result = builder.add_usd(stage)
    mapping = result['path_shape_map']
    for path, (ct, ca) in shapes.items():
        if path in mapping:
            builder.shape_collision_group[mapping[path]] = 0 if ct == ca == 0 else 1
    entries = [(path, mask) for path, mask in shapes.items() if path in mapping]
    # Bounded exact pair exclusion table, deliberately no optimization claim.
    for i, (p, (ct, ca)) in enumerate(entries):
        for q, (dt, da) in entries[i+1:]:
            if not ((ct & da) or (dt & ca)):
                builder.shape_collision_filter_pairs.append(tuple(sorted((mapping[p], mapping[q]))))
    for ends, values in pairs:
        if any(e not in mapping for e in ends):
            raise ValueError('Newton pruned a required collision endpoint')
        newton.solvers.mujoco.add_contact_pair(builder, mapping[ends[0]], mapping[ends[1]], **values)
    return result
