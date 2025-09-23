using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace JarvenFramework.FSM
{
    public interface IFSMState
    {
        IFSMState this[string trigger] { get; set; }

        string Name { get; set; }

        void OnEnter(string trigger, params object[] keys);

        void OnExit(string trigger);

        void OnStay();
    }
}