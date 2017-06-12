# Cpp Coding Guide

## Brief Guide
### Naming
#### under_scored
* Packages 
* Topics / Services
* Files
* Libraries
* Variables
* Member variables, with a** trailing underscore added**.
* Global variables, with a **leading g_ added**. **Discouraged.**
* Namespaces
#### CamelCased
* Classes / Types
#### camelCased
* Function / Methods
#### ALL_CAPITALS
* Constants

### Formatting
Indent each block by 2 spaces. Never insert literal tab characters.
The contents of a namespace are not indented.
Braces, both open and close, go on their own lines
### Header
All headers must be protected against multiple inclusion by #ifndef guards.
### Others
* Documentation  ** doxygen**, english
* Console output rosconsole
* Macros
Avoid preprocessor macros whenever possible. Unlike inline functions and const variables, macros are neither typed nor scoped.
* Using std::list std::vector std::cout std::endl instead of using namespace std;
* uint -> unsigned int,   isnan -> std::isnan()
* Assertions  ROS_ASSERT  ROS_BREAK()
* Delete redundancy blank spaces, lines, comments
* Comment accuracy

## Cpp Style Guide
Ref: http://wiki.ros.org/CppStyleGuide
### Target
* Clean
* Consistent

### Naming
The following shortcuts are used in this section to denote naming schemes:

* **CamelCased**: The name starts with a capital letter, and has a capital letter for each new word, with no underscores.
* ** camelCased**: Like CamelCase, but with a lower-case first letter
* **under_scored**: The name uses only lower-case letters, with words separated by underscores. (yes, I realize that under_scored should be underscored, because it's just one word).
* **ALL_CAPITALS**: All capital letters, with words separated by underscores.

#### Packages
ROS packages are under_scored.

#### Topics / Services
ROS topics and service names are under_scored.

#### Files
All files are **under_scored**.

Source files have the extension** .cpp**.

Header files have the extension **.h**.

Be descriptive, e.g., instead of l**aser.cpp**, use **hokuyo_topurg_laser.cpp**.

If the file primarily implements a class, name the file after the class. For example the class ActionServer would live in the file action_server.h.

##### Libraries
Libraries, being files, are u**nder_scored.**

Don't insert an underscore immediately after the lib prefix in the library name.

E.g.,
``` C++
lib_my_great_thing  ## Bad
libmy_great_thing  ## Good
```

#### Classes / Types
Class names (and other type names) are **CamelCased**

E.g.:
``` C++
class ExampleClass;
``` 
Exception: if the class name contains a short acronym, the acronym itself should be all capitals, e.g.:
``` C++
class HokuyoURGLaser;
``` 
Name the class after what it is. If you can't think of what it is, perhaps you have not thought through the design well enough.

Compound names of over three words are a clue that your design may be unnecessarily confusing.

#### Function / Methods
In general, function and class method names are camelCased, and arguments are **under_scored**, e.g.:
``` C++
int exampleMethod(int example_arg);
``` 
Functions and methods usually perform an action, so their name should make clear what they do: checkForErrors() instead of errorCheck(), dumpDataToFile() instead of dataFile(). Classes are often nouns. By making function names verbs and following other naming conventions programs can be read more naturally.

#### Variables
In general, variable names are **under_scored**.

Be reasonably descriptive and try not to be cryptic. Longer variable names don't take up more space in memory, I promise.

Integral iterator variables can be very short, such as **i, j, k**. Be consistent in how you use iterators (e.g., **i** on the outer loop, **j **on the next inner loop).

STL iterator variables should indicate what they're iterating over, e.g.:

``` C++
std::list<int> pid_list;
std::list<int>::iterator pid_it;
``` 
Alternatively, an STL iterator can indicate the type of element that it can point at, e.g.:

``` C++
std::list<int> pid_list;
std::list<int>::iterator int_it;
``` 
##### Constants

Constants, wherever they are used, are **ALL_CAPITALS**.

##### Member variables

Variables that are members of a class (sometimes called fields) are **under_scored**, with a trailing underscore added.

E.g.:

``` C++
int example_int_;
``` 

##### Namespaces
Namespace names are **under_scored**.

### Formatting
Your editor should handle most formatting tasks. See EditorHelp for example editor configuration files.

Indent each block by** 2 spaces**. **Never insert literal tab characters.**

**The contents of a namespace are not indented**.

Braces, both open and close, go on their own lines (no "cuddled braces"). E.g.:

``` C++
if(a < b)
{
  // do stuff
}
else
{
  // do other stuff
}
``` 
Braces can be omitted if the enclosed block is a single-line statement, e.g.:

``` C++
if(a < b)
  x = 2*a;
  ``` 
Always include the braces if the enclosed block is more complex, e.g.:

``` C++
if(a < b)
{
  for(int i=0; i<10; i++)
    PrintItem(i);
}
``` 
Here is a larger example:
``` C++
/*
 * A block comment looks like this...
 */
#include <math.h>
class Point
{
public:
  Point(double xc, double yc) :
    x_(xc), y_(yc)
  {
  }
  double distance(const Point& other) const;
  int compareX(const Point& other) const;
  double x_;
  double y_;
};
double Point::distance(const Point& other) const
{
  double dx = x_ - other.x_;
  double dy = y_ - other.y_;
  return sqrt(dx * dx + dy * dy);
}
int Point::compareX(const Point& other) const
{
  if (x_ < other.x_)
  {
    return -1;
  }
  else if (x_ > other.x_)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
namespace foo
{
int foo(int bar) const
{
  switch (bar)
  {
    case 0:
      ++bar;
      break;
    case 1:
      --bar;
    default:
    {
      bar += bar;
      break;
    }
  }
}
} // end namespace foo
``` 
##### Line length
Maximum line length is 120 characters.

#### #ifndef guards
All headers must be protected against multiple inclusion by #ifndef guards, e.g.:

``` C++
#ifndef PACKAGE_PATH_FILE_H
#define PACKAGE_PATH_FILE_H
...
#endif
``` 
This guard should begin immediately after the license statement, before any code, and should end at the end of the file.

### Documentation
Code must be documented. Undocumented code, however functional it may be, cannot be maintained.

We use ***doxygen*** to auto-document our code. Doxygen parses your code, extracting documentation from specially formatted comment blocks that appear next to functions, variables, classes, etc. Doxygen can also be used to build more narrative, free-form documentation.

See the **rosdoc **page for examples of inserting doxygen-style comments into your code.

All functions, methods, classes, class variables, enumerations, and constants should be documented.

### Console output
Avoid printf and friends (e.g., cout). Instead, use rosconsole for all your outputting needs. It offers macros with both printf- and stream-style arguments. Just like printf, rosconsole output goes to screen. Unlike printf, rosconsole output is:

* color-coded
* controlled by verbosity level and configuration file
* published on /rosout, and thus viewable by anyone on the network (only when working with roscpp)
* optionally logged to disk
### Macros
Avoid preprocessor macros whenever possible. Unlike inline functions and const variables, macros are neither typed nor scoped.

### Preprocessor directives (#if vs. #ifdef)
For conditional compilation (except for the #ifndef guard explained above), always use #if, not #ifdef.

Someone might write code like:

``` C++
#ifdef DEBUG
        temporary_debugger_break();
#endif
``` 
Someone else might compile the code with turned-off debug info like:

``` C++
cc -c lurker.cpp -DDEBUG=0
``` 
Always use #if, if you have to use the preprocessor. This works fine, and does the right thing, even if DEBUG is not defined at all.

``` C++
#if DEBUG
        temporary_debugger_break();
#endif
``` 

### Output arguments
Output arguments to methods / functions (i.e., variables that the function can modify) are passed by pointer, not by reference. E.g.:
``` C++
int exampleMethod(FooThing input, BarThing* output);
``` 
By comparison, when passing output arguments by reference, the caller (or subsequent reader of the code) can't tell whether the argument can be modified without reading the prototype of the method.

### Namespaces
Use of namespaces to scope your code is encouraged. Pick a descriptive name, based on the name of the package.

Never use a using-directive in header files. Doing so pollutes the namespace of all code that includes the header.

It is acceptable to use using-directives in a source file. But it is preferred to use using-declarations, which pull in only the names you intend to use.

E.g., instead of this:
``` C++
using namespace std; // Bad, because it imports all names from std::
``` 
Do this:
``` C++
using std::list;  // I want to refer to std::list as list
using std::vector;  // I want to refer to std::vector as vector
``` 

### Inheritance
Inheritance is the appropriate way to define and implement a common interface. The base class defines the interface, and the subclasses implement it.

Inheritance can also be used to provide common code from a base class to subclasses. This use of inheritance is discouraged. In most cases, the "subclass" could instead contain an instance of the "base class" and achieve the same result with less potential for confusion.

When overriding a virtual method in a subclass, always declare it to be virtual, so that the reader knows what's going on.

Multiple inheritance is strongly discouraged, as it can cause intolerable confusion.

### Exceptions
Exceptions are the preferred error-reporting mechanism, as opposed to returning integer error codes.

Always document what exceptions can be thrown by your package, on each function / method.

Don't throw exceptions from destructors.

Don't throw exceptions from callbacks that you don't invoke directly.

If you choose in your package to use error codes instead of exceptions, use only error codes. **Be consistent**.


### Enumerations
Namespaceify your enums, e.g.:

``` C++
namespace choices
{
  enum Choice
  {
     Choice1,
     Choice2,
     Choice3
  };
}
typedef choices::Choice Choice;
``` 
This prevents enums from polluting the namespace they're inside. Individual items within the enum are referenced by: Choices::Choice1, but the typedef still allows declaration of the Choice enum without the namespace.

### Globals
Globals, both variables and functions, are **discouraged**. They pollute the namespace and make code less reusable.

Global variables, in particular, are strongly discouraged. They prevent multiple instantiations of a piece of code and make multi-threaded programming a nightmare.

Most variables and functions should be declared inside classes. The remainder should be declared inside namespaces.

**Exception:** a file may contain a **main() function and a handful of small helper functions **that are global. But keep in mind that one day those helper function may become useful to someone else.


### Static class variables
Static class variables are** discouraged.** They prevent multiple instantiations of a piece of code and make multi-threaded programming a nightmare.

### Calling exit()
Only call **exit() **at a well-defined exit point for the application.

Never call **exit() **in a library.

### Assertions
Use assertions to check preconditions, data structure integrity, and the return value from a memory allocator. Assertions are better than writing conditional statements that will rarely, if ever, be exercised.

Don't call assert() directly. Instead use one of these functions, declared in ros/assert.h (part of the rosconsole package):
``` C++
/** ROS_ASSERT asserts that the provided expression evaluates to
 * true.  If it is false, program execution will abort, with an informative
 * statement about which assertion failed, in what file.  Use ROS_ASSERT
 * instead of assert() itself.
 */
#define ROS_ASSERT(expr) ...
``` 

``` C++
/** ROS_BREAK aborts program execution, with an informative
 * statement about which assertion failed, in what file. Use ROS_BREAK
 * instead of calling assert(0) or ROS_ASSERT(0).
 */
#define ROS_BREAK() ...
``` 
Do not do work inside an assertion; only check logical expressions. Depending on compilation settings, the assertion may not be executed.

### Testing
See gtest.

### Portability
We're currently support Linux and OS X, with plans to eventually support other OS's, including possibly Windows. To that end, it's important to keep the C++ code portable. Here are a few things to watch for:

* Don't use uint as a type. Instead use unsigned int.
* Call isnan() from within the std namespace, i.e.: std::isnan()
### Deprecation
To deprecate an entire header file within a package, you may include an appropriate warning:
``` C++
#warning mypkg/my_header.h has been deprecated
``` 
To deprecate a function, add the deprecated attribute:

``` C++
ROS_DEPRECATED int myFunc();
``` 
To deprecate a class, deprecate its constructor and any static functions:
``` C++
class MyClass
{
public:
  ROS_DEPRECATED MyClass();

  ROS_DEPRECATED static int myStaticFunc(); 
};
```
