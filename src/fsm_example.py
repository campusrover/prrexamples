# Example of finite state machines
# First we show how to do the example using the pytransitions package
# Make sure the package is installed: pip install transitions; pip install transitions[diagrams]  
#
# In ipython:
#     from fsm_example import MoveBaseRecovery
#     t = MoveBaseRecovery("base")
#     t.state
#     t.stuck() # etc
#     t.get_graph().draw('fsm_example.png', prog='dot')

from transitions.extensions import GraphMachine as Machine

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
