"""Intentional waiting compares two safe actions; subsequent contention is LP."""

from math import isclose
from statistics import mean,pstdev

from examples.RMFS.Tests.run_wait_or_go import run,experiment,SEND,WAIT


def test_same_state_prefers_wait_for_makespan_but_send_for_mean_completion():
    maximum=run('short_A','lookahead_makespan')
    average=run('short_A','lookahead_mean')
    assert maximum['completed'] and average['completed']
    assert maximum['decisions'][0]['action']==WAIT
    assert average['decisions'][0]['action']==SEND
    assert maximum['makespan']<average['makespan']
    assert average['mean_completion']<maximum['mean_completion']
    for row in (maximum,average):
        values=list(row['completion_times'].values())
        assert row['mean_completion']==mean(values)
        assert row['completion_std']==pstdev(values)
        assert row['makespan']==max(values)


def test_wait_is_not_unconditionally_good_even_with_longer_later_path():
    row=run('long_A','lookahead_makespan')
    branches={b['action']:b for b in row['decisions'][0]['branches']}
    assert branches[SEND]['completed'] and branches[WAIT]['completed']
    assert row['decisions'][0]['action']==SEND
    assert branches[SEND]['makespan']<branches[WAIT]['makespan']


def test_real_decisions_can_switch_actions_without_nested_lookahead():
    row=run('mixed_four','lookahead_makespan')
    assert row['completed'] and not row['nested_lookahead']
    assert [d['action'] for d in row['decisions']]==[WAIT,SEND]
    assert row['lookahead_calls']==2 and row['rollouts']==4
    assert row['decisions'][0]['branches'][0]['later_actions']
    for decision in row['decisions']:
        for branch in decision['branches']:
            assert all(action['action']==SEND for action in branch['later_actions'])
    assert all(event['policy']=='longest_remaining_distance' for event in row['choice_history'])
    for policy in ('fifo','length'):
        assert row['makespan']<run('mixed_four',policy)['makespan']


def test_random_repeats_and_every_distinct_sequence_has_collision_stop_validation():
    result=experiment('mixed_four',repetitions=6)
    random=next(p for p in result['policies'] if p['policy']=='random')
    assert random['repetitions']==6
    for trial in random['trials']:
        assert trial['completed'] and trial['validation']['contacts']==0
        assert trial['validation']['stop_on_collision']
        repeated=run('mixed_four','random',trial['seed'])
        assert repeated['makespan']==trial['makespan']
        assert [d['action'] for d in repeated['decisions']]==[d['action'] for d in trial['decisions']]
    for key in ('makespan','mean_completion','completion_std'):
        values=[t[key] for t in random['trials']]
        assert isclose(random['metrics'][key]['mean'],mean(values))
        assert isclose(random['metrics'][key]['across_run_std'],pstdev(values))


def test_production_controller_has_no_wait_or_go_adapter():
    from examples.RMFS.controlled import create_controlled_model
    from examples.RMFS.TrafficManager import TrafficManager
    model,_,_=create_controlled_model({'R1':((0,0),(1,0))},collision_check=False)
    assert type(model.controller.traffic_manager) is TrafficManager
    assert '?wait_action' not in model.controller.IP
