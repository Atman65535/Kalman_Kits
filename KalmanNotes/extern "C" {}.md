### **Main Reason**
When we use cpp for coding, our function symbols are follow *Itanium C++ ABI*, which encode our function names into a weird symbol (for reloading, namespaces and ...) 
 ~~~cpp
 _ZN4eskf8predictEv
 ~~~
 - \_Z means C++ mangled name
 - N nested, in a namespace or class
 - 4eskf, name length is 4, name eskf
 - 7predict, name length is 8, predict
 - End of name
 - v void return value.

In C language, we just have `predict` as our symbol name. So `undefined symbol` error happens when we include C header in c++ file.

We must divide two things: Compile and linking. When we compile our code, we use C compiler for c codes, cpp compiler for cpp codes. The Cpp compiler will generate complicated alias for a function while the C compiler doesn't. If we don't add extern C, c++ compiler will regard our function invoke as c++ invoking, and decide the name for linking
必须说清楚编译和链接，编译是同时使用C和C++编译器的，在处理c文件的时候c编译器生成简单名字，相应的c++编译器生成复杂的重载描述的等等。如果在C++文件里面我们调用了一个函数名，声明了它是外部C函数的话可以生成简单的搜索名字，否则会有复杂名字，当然这就找不到了。