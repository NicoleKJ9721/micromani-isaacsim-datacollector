import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'isaacsim_tools'))
from micromani_step_rate import select_physics_hz

def test_near_table_contact_and_closing_always_use_precision():
    assert select_physics_hz(.3,[0,0,0,0],False)==120
    assert select_physics_hz(.215,[0,0,0,0],False)==960
    assert select_physics_hz(.3,[.001,0,0,0],False)==960
    assert select_physics_hz(.3,[0,0,0,0],True)==960
    assert select_physics_hz(float('nan'),[0,0,0,0],False)==960
