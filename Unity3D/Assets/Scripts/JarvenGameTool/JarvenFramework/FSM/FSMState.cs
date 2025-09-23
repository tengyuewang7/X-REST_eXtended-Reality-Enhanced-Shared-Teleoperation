using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace JarvenFramework.FSM
{
    public abstract class FSMState : IFSMState
    {
        private Dictionary<string, IFSMState> _transitionDic = new Dictionary<string, IFSMState>();
        public string Name { get; set; }
        public IFSMState this[string trigger]
        {
            get
            {
                IFSMState res = null;

                _transitionDic?.TryGetValue(trigger, out res);

                return res;
            }
            set
            {
                if (_transitionDic != null)
                {
                    _transitionDic[trigger] = value;
                }
            }
        }
        public abstract void OnEnter(string trigger, params object[] keys);
        public abstract void OnExit(string trigger);
        public abstract void OnStay();
    }
}
