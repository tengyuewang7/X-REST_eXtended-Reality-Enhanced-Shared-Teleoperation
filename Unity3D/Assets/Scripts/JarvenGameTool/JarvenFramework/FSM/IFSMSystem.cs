using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace JarvenFramework.FSM
{
    public interface IFSMSystem
    {
        IFSMState AnyState { get; }
        IFSMState CurState { get; }
        void AddState(IFSMState state, bool isDefault = false);
        void SetTrigger(string trigger, params object[] keys);
        void TurnToDefault();
        void Update();
    }
}