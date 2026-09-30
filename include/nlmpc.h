#include "mpc/NLMPC.hpp"

using namespace std;

template<typename FnState, typename FnObj>
class NLMPC
{
	public:
		NLMPC();
		template<typename t>
		NLMPC(int _nx, int _ny, int _nu, int _nph, int _nch, int _nieq, int _neq,
			function<FnState> _stateEq, function<FnObj> _objEq);
            // function<void(mpc::cvec<>,const mpc::cvec<>,const mpc::cvec<>, const unsigned int)> _stateEq, function<void(mpc::cvec<>,mpc::cvec<>,mpc::cvec<>)> _objEq);
		// function<void(mpc::cvec<>,const mpc::cvec<>,const mpc::cvec<>, const unsigned int)> stateEq;
		function<FnState> stateEq;
		function<FnObj> objEq;
		// void createParams(t);
	private:
		mpc::NLMPC<> controller;
		int Nx;
		int Ny;
		int Nu;
		int Nph;
		int Nch;
		int Nieq;
		int Neq;
};	