"""firmware/config.json and the per-model examples next to it."""

import pytest

from fakes import ROBOTS, load_config

# keys that are allowed to differ between models
PER_MODEL = {"robot", "re", "origin", "hop_arc_height"}


@pytest.mark.parametrize("model", sorted(ROBOTS))
def test_example_has_every_key_config_json_has(model):
    assert set(load_config(model)) == set(load_config())


@pytest.mark.parametrize("model", sorted(ROBOTS))
def test_example_is_labelled_with_its_model(model):
    assert load_config(model)["robot"] == model


def test_examples_differ_only_in_model_geometry():
    t3, t3plus = load_config("tapster3"), load_config("tapster3plus")
    assert {k for k in t3 if t3[k] != t3plus[k]} <= PER_MODEL
