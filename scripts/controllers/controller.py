import abc


class Controller:
    """ Abstract Base Class for control implementation. """
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def solve(self):
        """ Returns a dictionary sol_dict with control input to apply,
            as well as other useful information (e.g. MPC solution).
            
            In particular, sol_dict must have a key "u_control" such that
			sol_dict["u_control"][0] = acceleration input to apply
			sol_dict["u_control"][1] = steering input to apply            
        """
        raise NotImplementedError

    @abc.abstractmethod
    def update(self, update_dict):
        """ Updates the state of the controller with feedback info contained in update_dict. """
        raise NotImplementedError
