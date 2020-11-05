import abc

class Controller(object):
	__metaclass__ = abc.ABCMeta

	@abc.abstractmethod
	def solve():
		""" Do stuff """
		return

	@abs.abstractmethod
	def update():
		""" Do stuff """
		return