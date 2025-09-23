using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace JarvenFramework.FSM
{
    public sealed class FSMSystem : IFSMSystem
    {
        private readonly List<IFSMState> _stateList = new List<IFSMState>();

        private IFSMState _curState;

        private IFSMState _defaultState;

        private IFSMState _anyState = new AnyState();

        public IFSMState AnyState => _anyState;

        public IFSMState CurState => _curState;

        public void AddState(IFSMState state, bool isDefault = false)
        {
            if (state != null)
            {
                _stateList.Add(state);

                if (isDefault)
                {
                    _defaultState = state;

                    _curState = state;

                    state.OnEnter(string.Empty);
                }
            }
        }

        public void TurnToDefault()
        {
            TurnState(string.Empty, _curState, _defaultState);
        }


        public void SetTrigger(string trigger, params object[] keys)
        {
            IFSMState targetState = _anyState[trigger];

            if (targetState == null && _curState != null)
            {
                targetState = _curState[trigger];
            }

            TurnState(trigger, _curState, targetState, keys);
        }

        public void Update()
        {
            _curState?.OnStay();
        }

        private void TurnState(string trigger, IFSMState oldState, IFSMState newState, params object[] keys)
        {
            if (newState != null && _stateList.Contains(newState))
            {
                oldState?.OnExit(trigger);

                _curState = newState;

                newState.OnEnter(trigger, keys);
            }
        }
    }
}
