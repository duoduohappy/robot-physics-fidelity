"""Identity-aligned observations, not a proof of physical equivalence."""
import hashlib
import numbers
import numpy as np
import mujoco
from . import engine, contract


def fingerprint(model):
    h = hashlib.sha256()
    count = 0
    for prefix, obj in [('model', model), ('option', model.opt)]:
        for name in sorted(n for n in dir(obj) if not n.startswith('_')):
            value = getattr(obj, name)
            if isinstance(value, bytes):
                payload = value
            elif isinstance(value, np.ndarray):
                payload = repr((value.dtype.str, value.shape)).encode() + value.tobytes()
            elif isinstance(value, (numbers.Number, bool)):
                payload = repr(value).encode()
            else:
                continue
            label = (prefix + '.' + name).encode()
            for part in (label, payload):
                h.update(len(part).to_bytes(8, 'little')); h.update(part)
            count += 1
    return {'sha256': h.hexdigest(), 'public_fields': count,
            'scope': 'public numeric arrays/scalars and bytes exposed by pinned MuJoCo; not external files or a cryptographic attestation'}


def audit(a, b):
    maps = engine.alignment(a, b)
    rows = []

    def record(group, field, name, left, right, quaternion=False):
        x, y = np.asarray(left), np.asarray(right)
        if quaternion and np.dot(x, y) < 0:
            y = -y
        finite = bool(np.isfinite(x).all() and np.isfinite(y).all())
        error = float(np.max(np.abs(x.astype(float)-y.astype(float)))) if x.size else 0.
        ok = finite and bool(np.allclose(x, y, rtol=1e-6, atol=1e-8))
        rows.append({'group': group, 'field': field, 'entity': name, 'match': ok,
                     'max_absolute_difference': error if finite else None,
                     'source': x.tolist(), 'target': y.tolist()})

    groups = [
        ('body', 'inertia', ['body_mass', 'body_ipos']),
        ('body', 'body_frame', ['body_pos', 'body_quat']),
        ('joint', 'joint', ['jnt_type', 'jnt_pos', 'jnt_axis', 'jnt_limited', 'jnt_range',
                             'jnt_solref', 'jnt_solimp', 'jnt_margin', 'jnt_actfrclimited', 'jnt_actfrcrange']),
        ('dof', 'joint_dynamics', ['dof_damping', 'dof_armature', 'dof_frictionloss']),
        ('actuator', 'actuator', ['actuator_gaintype', 'actuator_biastype', 'actuator_dyntype',
                                 'actuator_gainprm', 'actuator_biasprm', 'actuator_gear',
                                 'actuator_ctrllimited', 'actuator_ctrlrange',
                                 'actuator_forcelimited', 'actuator_forcerange']),
        ('geom', 'collision', ['geom_type', 'geom_size', 'geom_pos', 'geom_quat',
                              'geom_contype', 'geom_conaffinity', 'geom_friction',
                              'geom_margin', 'geom_gap', 'geom_condim'])]
    for mapping, group, fields in groups:
        for i, j in maps[mapping].items():
            name = f'{mapping}[{i}]'
            kind = {'body': mujoco.mjtObj.mjOBJ_BODY, 'joint': mujoco.mjtObj.mjOBJ_JOINT,
                    'geom': mujoco.mjtObj.mjOBJ_GEOM, 'actuator': mujoco.mjtObj.mjOBJ_ACTUATOR}.get(mapping)
            if kind is not None:
                name = mujoco.mj_id2name(a, kind, int(i)) or name
            for field in fields:
                record(group, field, name, getattr(a, field)[i], getattr(b, field)[j], field.endswith('_quat'))
    # Compare inertia tensors rather than principal-axis quaternion representations.
    def tensor(m, i):
        mat = np.zeros(9)
        mujoco.mju_quat2Mat(mat, m.body_iquat[i])
        r = mat.reshape(3, 3)
        return r @ np.diag(m.body_inertia[i]) @ r.T
    for i, j in maps['body'].items():
        record('inertia', 'local_inertia_tensor', mujoco.mj_id2name(a,mujoco.mjtObj.mjOBJ_BODY,i), tensor(a,i), tensor(b,j))
    for i in range(a.npair):
        ends = {maps['geom'][int(a.pair_geom1[i])], maps['geom'][int(a.pair_geom2[i])]}
        found = [j for j in range(b.npair) if {int(b.pair_geom1[j]),int(b.pair_geom2[j])} == ends]
        if len(found) != 1:
            rows.append({'group':'pair','field':'identity','entity':str(i),'match':False,'reason':'missing or ambiguous pair'})
        else:
            for field, _, _ in contract.FIELDS.values():
                record('pair', field, str(i), getattr(a,field)[i],getattr(b,field)[found[0]])
    record('pair', 'count', 'model', a.npair, b.npair)
    for field in engine.SETTINGS:
        record('configuration', field, 'option', getattr(a.opt,field),getattr(b.opt,field))
    differences = [r for r in rows if not r['match']]
    for row in differences:
        field=row['field']
        row['review_class']='physical_parameter_or_representation_difference'
        if row['group']=='configuration':row['review_class']='observed_before_explicit_test_override'
        elif field in ['geom_friction','geom_condim','geom_margin','geom_gap']:
            row['review_class']='geom_default_distinct_from_explicit_pair_parameters'
        elif field=='jnt_actfrcrange':row['review_class']='force_range_must_be_interpreted_with_enable_flag'
        elif field in ['body_pos','body_quat','jnt_pos','jnt_axis']:
            row['review_class']='frame_representation_requires_joint_context'
    return {'status':'MATCH_WITHIN_AUDIT_TOLERANCE' if not differences else 'SEMANTIC_DIFFERENCES',
            'checked_rows':len(rows),'difference_count':len(differences),'differences':differences,
            'source_fingerprint':fingerprint(a),'target_fingerprint':fingerprint(b),
            'uncovered':'automatic contact endpoints outside contract; arbitrary USD schemas; physical equivalence of alternative frame decompositions',
            'interpretation':'Differences are review candidates, not proven causes; solver settings are observed before evaluator overrides them.'}
