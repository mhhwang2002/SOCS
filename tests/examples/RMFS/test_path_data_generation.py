"""Seeded data acceptance is geometric and untimed, never performance filtered."""

from dataclasses import replace
import json
from pathlib import Path

import pytest

from examples.RMFS.Tests.generate_path_data import Config,generate,load_witness,measure,metric_rejection


def test_seeded_reproducibility_and_stable_sample_prefix():
    config=Config(grid_size=8,objects=3,samples=2,min_edges=2,max_edges=10,seed=41)
    a,b=generate(config),generate(config)
    assert len(a['samples'])==2
    assert a['dataset_hash']==b['dataset_hash'] and a['samples']==b['samples']
    bigger=generate(replace(config,samples=3))
    assert [s['path_hash'] for s in a['samples']]==[s['path_hash'] for s in bigger['samples'][:2]]


@pytest.mark.parametrize('name',('simple','complex','random'))
def test_generated_datasets_round_trip_and_all_acceptance_conditions(name):
    filename=Path(__file__).resolve().parents[3]/'examples'/'RMFS'/'Tests'/'data'/'generated'/f'{name}.json'
    data=json.loads(filename.read_text(encoding='utf-8'))
    cfg=Config(**data['config'])
    assert len(data['samples'])==data['summary']['accepted']==10
    assert data['no_policy_performance_filter']
    for sample in data['samples']:
        solver,witness=load_witness(data,sample['index'])
        assert solver.validate(witness)
        actual=measure(dict(zip(solver.names,solver.paths)),solver.footprints[0],solver.policy)
        assert all(pair['F_ij']==1 for pair in actual['pairs'])
        assert metric_rejection(actual,cfg) is None
        assert actual['directed_pairwise_interval_count']==sample['metrics']['directed_pairwise_interval_count']
        assert actual['mean_pairwise_interval_m']==sample['metrics']['mean_pairwise_interval_m']
        assert set(sample['serial_execution_order'])==set(sample['object_names'])
        assert all(len(route)==len(set(tuple(p) for p in route)) for route in sample['routes'].values())


def test_budget_exhaustion_is_not_an_infeasibility_claim():
    data=generate(Config(grid_size=6,objects=2,samples=1,min_edges=2,max_edges=6,
                         min_intervals=10000,scenario_attempts=2,candidates_per_object=5,candidate_budget=20))
    assert not data['samples'] and data['summary']['not_found']==1
    assert data['audit'][0]['status'] in ('budget_exhausted','scenario_budget_exhausted')
    assert 'infeasible' not in data['audit'][0]['status']


def test_corrupt_witness_and_inconsistent_geometry_are_rejected():
    data=generate(Config(grid_size=8,objects=2,samples=1,min_edges=2,max_edges=10,seed=3))
    damaged=json.loads(json.dumps(data))
    damaged['samples'][0]['witness']['consumed_counts'][-1][0]=0
    with pytest.raises(ValueError):
        load_witness(damaged)
    damaged=json.loads(json.dumps(data))
    name=damaged['samples'][0]['object_names'][0]
    damaged['samples'][0]['paths_m'][name][0][0]+=.1
    with pytest.raises(ValueError):
        load_witness(damaged)


def test_invalid_configuration_is_rejected_before_search():
    with pytest.raises(ValueError):
        Config(min_intervals=5,max_intervals=2)
    with pytest.raises(ValueError):
        Config(min_interval_m=5,max_interval_m=2)
    with pytest.raises(ValueError):
        Config(spacing_m=0)
