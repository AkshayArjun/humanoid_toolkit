# Implementation of ABC for the code base.

### what are ABC?

Abc stands for abstract base classes. its a type of class that from my understadning serves as a templaate of functions to be implemented . subclasses can inherit the abc and overwrite the method with their own defenition. However, all @abstractmethods need to be filled else we will endup with and error ; when trying to create an object with the sub class 'TypeError'

abc is a contract ie mandator code + shared code (the concrete methods).
