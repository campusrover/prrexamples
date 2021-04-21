# Example of finite state machines
# First we show how to do the example using the pytransitions package
# Make sure the package is installed: pip install transitions
#
# In ipython: `from fsm_example import MoveBaseRecovery`
from transitions import Machine

class MoveBaseRecovery(object):
  states = ['navigating', 'reset', 'rotate1', 'reset_aggressive', 'rotate2', 'abort']

  def __init__(self, name):
    self.name = name
    self.machine = Machine(model=self, states=MoveBaseRecovery.states, initial='navigating')

    self.machine.add_transition(trigger='stuck', source='navigating', dest='reset')
    self.machine.add_transition(trigger='clear', source='reset', dest='navigating')
    self.machine.add_transition(trigger='clear', source='rotate1', dest='navigating')
    self.machine.add_transition(trigger='clear', source='reset_aggressive', dest='navigating')
    self.machine.add_transition(trigger='clear', source='rotate2', dest='navigating')
    self.machine.add_transition(trigger='stuck', source='reset', dest='rotate1')
    self.machine.add_transition(trigger='stuck', source='rotate1', dest='reset_aggressive')
    self.machine.add_transition(trigger='stuck', source='reset_aggressive', dest='rotate2')
    self.machine.add_transition(trigger='stuck', source='rotate2', dest='abort')
