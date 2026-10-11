"""plain_copy: the same independent copy as copy.deepcopy for the loop's plain data, and deepcopy for anything else."""
from collections import namedtuple
from copy import deepcopy

from robot_platform.control.plain_data import plain_copy

Sample = namedtuple('Sample', 'positions sequence')


def nested():
    return {'joints': [{'name': 'left_arm_j1', 'kp': 150., 'window': [-.03, .03], 'follow': True, 'travel': None}],
            'reference': (0., (1, 2), 'rad'), 'count': 3}


def test_matches_deepcopy_and_shares_nothing_with_the_original():
    original = nested()
    copied = plain_copy(original)
    assert copied == deepcopy(original) == original
    assert type(copied['reference']) is tuple and type(copied['reference'][1]) is tuple
    copied['joints'][0]['window'][0] = -1.
    copied['joints'][0]['kp'] = 0.
    copied['joints'].append({})
    assert original == nested()
    assert copied['joints'] is not original['joints'] and copied['joints'][0] is not original['joints'][0]


def test_anything_but_plain_containers_goes_through_deepcopy():
    sample = Sample([1., 2.], 7)
    copied = plain_copy({'leader': sample})
    assert type(copied['leader']) is Sample and copied['leader'] == sample
    copied['leader'].positions.append(3.)
    assert sample.positions == [1., 2.]

    class Box:
        def __init__(self):
            self.items = [1]
    box = Box()
    other = plain_copy([box])[0]
    assert other is not box and other.items == [1] and other.items is not box.items
