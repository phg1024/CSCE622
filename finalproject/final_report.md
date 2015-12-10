<div align="center">
<h3>CSCE 622: Generic Programming -- Final Project Report</h3>
<h4>Peihong Guo UIN: 421003404</h4>
</div>

### Project title: Half-edge Data Structure using BGL

#### Brief Summary of Goals
The main goal of this project is to implement a light-weight generic half-edge data structure using BGL. Half-edge data structure is an important data structure for representing and manipulating 3D mesh data in computational geometry. A generic implementation of half-edge data structure could be used in various computational geometry algorithms, including mesh simplification, repair and constructive solid geometry, that rely on this data structure. We will demonstrate the power of this generic implementation using sample algorithms such as mesh simplification and Laplacian deformation.

#### User's Guide
The User's guide should begin with an overview section. The overview section is a crucial part of the documentation and you should make sure it really does give an overview. Before writing the overview, consider the following issues:

Who is the intended audience.
In particular, the intended audience is not the course instructor. Rather, think of a professional programmer. It is a good idea to describe how much, and what kind, of knowledge you are assuming from the audience.

What is the right level of discussion for the intended audience.
Do not dive into too much detail in the overview, save that for later parts of your document.

What terms need to be defined to convey the main points to the intended audience.
After the overview, the user's guide should contain tutorial material, examples, discussions of normal usage, pitfalls, etc. You do not need to go into extremes with such material for the purposes of the class project, but provide enough to make your project accessible to others.

Include a bibliography of your sources (journals, web sites, your own previous projects, etc.) for concrete algorithms, data structures, or other information relevant to your software. It may make more sense to have a common bibliography for your entire document, rather than just for the User's guide. Use your judgment.

#### Reference Manual
See how to use the STL documentation for description of how, e.g., concepts are documented in the SGI STL documentation. Follow these guidelines as is appropriate for your project.

#### Design Document
Describe the most important design decisions. Why the interface is the way it is? Why were particular algorithms or data structures chosen?

One purpose of this section is to avoid extra work in the future. Unless the thinking behind various design decisions is documented, the same thinking process may have to be repeated later if the design is re-evaluated. In particular, documenting the reasons why other (inferior) choices were not selected is helpful.

#### Source code
Test for small cases, including all boundary cases you can think of, such as empty sequences, one-element sequences, graphs with no edges, graphs with more than one parallel edges, and so forth. Test for large cases as well; if possible, define an acceptance routine that checks correctness of result by some means.

If applicable, you should include timings (you want to make sure that the complexity guarantees that you documented in your reference manual are realistic). Test for large cases, measuring performance for large randomly generated inputs of different sizes. Document all necessary detail (hardware platform, compiler, optimization flags, …) to allow someone other than you to repeat the same measurements.

#### Description and results of testing

#### List of Sources
1. [Half edge data structure in CGAL](http://doc.cgal.org/latest/HalfedgeDS/index.html)
2. [Boost graph library](http://www.boost.org/doc/libs/1_59_0/libs/graph/doc/table_of_contents.html)
3. [OpenMesh library](http://www.openmesh.org/)
4. [Mesh simplification in CGAL]( http://doc.cgal.org/latest/Surface_mesh_simplification/)
5. [Mesh deformation]( http://www.cse.wustl.edu/~taoju/cse554/lectures/lect08_Deformation.pdf)
