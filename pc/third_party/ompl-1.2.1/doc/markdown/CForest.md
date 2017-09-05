# CForest Parallelization Framework

__Contents:__

- \ref cf_ompl
- \ref cf_diff
- \ref cf_example
- \ref cf_results
- \ref cf_advanced
  - \ref cf_implementation
  - \ref cf_limitations
  - \ref cf_compatible

CForest was proposed by M.Otte and N. Correll in [this paper.](http://www.mit.edu/~ottemw/html_stuff/pdf_files/otte_ieeetro2013.pdf)

The main idea behind CForest is that many trees are built in parallel between the same start and goal states. The key concepts of CForest are:

- Every time a tree finds a better solution, it is shared with all other trees so that all trees have the best solution found so far.
- Trees are expanded into regions that are known to be beneficial. Samples that cannot lead to a better solution are immediately discarded.
- Trees are pruned every time a better solution is found. Those states in the tree that do not help to find a better solution are removed from the tree.

CForest is designed to be used with any random tree algorithm under the following assumptions:

1. The search tree has almost sure convergence to the optimal solution.
2. The configuration space obeys the [triangle inequality.](http://en.wikipedia.org/wiki/Triangle_inequality) That is, there exists an admissible heuristic.

# CForest in OMPL {#cf_ompl}

CForest has been included into OMPL as a new [optimizing planner](optimalPlanning.html) called ompl::geometric::CForest.

\Note CForest is designed to optimize path lengths. In OMPL, it is possible to optimize with respect to an arbitrary optimization objective. Therefore, the requirements for CForest in OMPL are not for the _state space_ but for the _optimization objective_. It requires an optimization objective with an admissible heuristic. Since this is complex to check, a warning is shown if the state space is not a metric space (although this does not mean that the optimization objective is not valid to be used with CForest).

Currently RRT* (ompl::geometric::RRTstar) is the only underlying planner available, since it is the only single-query, incremental, asympotically optimal planning algorithm implemented in OMPL.

The CForest planner is responsible for coordinating different trees and sharing the solutions found. Path sharing is done through a specific CForest state sampler (ompl::base::CForestStateSampler). This sampler wraps around the default sampler associated with the state space CForest is planning in. The sampler wrapper will usually just pass through any calls to sampling methods, but when a thread finds a new best solution, subsequent calls to the sampling methods will return subsequent states along the best found path. This is done for each thread except the one that found the best solution.

From the user perspective, CForest can be used as any other planning algorithm:

~~~{.cpp}
#include <ompl/geometric/planners/cforest/CForest.h>
...
// Setting up the planning problem with SimpleSetup
SimpleSetup ss;
...
ompl::base::PlannerPtr planner(new ompl::geometric::CForest(ss.getSpaceInformation()));
ss.setPlanner(planner);
...
~~~

By default, it will use as many RRT* instances as cores available. The number of instances and the underlying planner can be modified calling the `addPlannerInstances<T>()` method:

~~~{.cpp}
ompl::base::PlannerPtr planner(new ompl::geometric::CForest(ss.getSpaceInformation()));
// Setting up CForest to expand 5 trees of RRTstar.
planner->as<ompl::geometric::CForest>()->addPlannerInstances<ompl::geometric::RRTstar>(5);
~~~

The call `addPlannerInstances<T>()` will check the `PlannerSpecs` of the planner type given. CForest only supports all those planners with the `canReportIntermediateSolutions` spec true. Otherwise, the following warning will appear during execution (test with PRM*):**

~~~{.cpp}
planner->as<ompl::geometric::CForest>()->addPlannerInstances<ompl::geometric::PRMstar>(2);
~~~
    Warning: PRMstar cannot report intermediate solutions, not added as CForest planner.
             at line 100 in ompl/geometric/planners/cforest/CForest.h

If not valid planners are added by the user, two instances of RRTstar will be automatically set.

Alternatively, only the number of threads could be specified and the default underlying planner (RRT*) will be chosen. This is specially useful when using the planner with a benchmark configuration file:

~~~{.cpp}
// Using 6 threads of the default planner
planner->as<ompl::geometric::CForest>()->setNumThreads(6);
~~~

\note No Python bindings are available for this planner due to its multithreaded implementation.

# Main differences with the paper version {#cf_diff}

When implementing CForest, the focus was to modify the underlying planner as little as possible. Although the main idea of CForest remains, the actual implementation differs from the one proposed in the paper:

- No message passing is used. Instead, shared memory and std::threads are employed.
- The paper creates two different versions: sequential (many trees expanding in the same CPU) and parallel (1 tree per CPU). Since std::threads are used, the trees/CPU division is done by the scheduler.
- In the paper, shared states are treated in a slightly different way than randomly sampled states. Due to the CForestStateSampler encapsulation, all states are treated the same way. Under some circumstances shared states are not included in other trees. However, tests showed that this does not have a high impact on performance.
- Sampling bounds are not explicitly set. When samples are created, we check whether they can lead to a better solution.
- Start and goal states are not included in the shared paths in order to keep code simpler.
- Before pruning a tree, it is checked how many states would be removed. If the ratio of the size of the new tree size to the size of the old tree is not small enough, pruning will not be carried out. This reduces the amortized cost of having to rebuild NearestNeighbors datastructures when states are pruned.

\note Despite all these differences, the CForest implementation greatly improves the performance of the underlying planner. However, an implementation closer to the one described in the paper could improve the performance. Please, take that into account if you plan to compare your algorithm against CForest.


# Example {#cf_example}

- [Circle Grid benchmark](CForestCircleGridBenchmark_8cpp_source.html). Benchmarks the perfomance of CForest against RRT*  in a specific 2D circle grid problem.

# Results {#cf_results}

CForest produces many interesting results. All these results are obtained with the alpha 1.5 puzzle benchmark configuration included in OMPL.app. The following figure shows the results of running CForest in 2 and 16 threads in a **16-core** machine. Also, the standard RRT* and the pruned version are included in the benchmark.

\htmlonly
<div class="row">
<div class="col-md-6 col-sm-6">
  <img src="images/cforest.png" width="100%"><br>
<b>Best cost evolution through time</b> CForest converges faster towards the minimum cost (optimal solution) as the threads are increased. Pruned RRT* also improves the standard RRT*.
</div>
<div class="col-md-6 col-sm-6">
  <img src="images/sharedpaths.png" width="100%"><br>
<b>Number of paths shared by CForest.</b>
</div>
</div>
\endhtmlonly

Another interesting experiment is to run CForest with pruning deactivated and compare it to the standard CForest:

\htmlonly
<div class="row">
  <img src="images/prunevsnoprune.png" width="100%"><br>
<b>Best cost evolution through time</b> Not pruning trees affects negatively on the convergence. However, it still improves the standard RRT.
</div>
\endhtmlonly

In case you only have one core available, CForest still improves the RRTstar performance! The following figure shows that using CForest in one single core but with many threads (in the picture 4 and 8) also improves the convergence rate against standard RRT* and its pruned version, reaching lower cost solutions in much less time.

\htmlonly
<div class="row">
  <img src="images/threadscforest.png" width="100%"><br>
<b>Best cost evolution through time</b> Sequential version of the CForest: many threads working in the same core.
</div>
\endhtmlonly


# Advanced information {#cf_advanced}

## Design and implementation details {#cf_implementation}

The CForest _planner_ comes with its own state sampler ompl::base::CForestStateSampler and its own state space ompl::base::CForestStateSpaceWrapper. They are completely transparent to the user as the ompl::geometric::CForest handles the creation of these.

CForest operates on the user-specified ompl::base::SpaceInformation. However, CForest instantiates the underlying planners with an individual SpaceInformation instance for each planner, containing an instance of a `CForestStateSpaceWrapper`. When the underlying planner allocates the `StateSampler`, `CForestStateSpaceWrapper` creates an instance of a `CForestStateSampler` which wraps the user-specified state sampler (or the default sampler if none was provided). This design is summarized in the following schema:

\htmlonly
<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<!-- Created with Inkscape (http://www.inkscape.org/) -->

<svg
   xmlns:dc="http://purl.org/dc/elements/1.1/"
   xmlns:cc="http://creativecommons.org/ns#"
   xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"
   xmlns:svg="http://www.w3.org/2000/svg"
   xmlns="http://www.w3.org/2000/svg"
   xmlns:xlink="http://www.w3.org/1999/xlink"
   xmlns:sodipodi="http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd"
   xmlns:inkscape="http://www.inkscape.org/namespaces/inkscape"
   width="555.25232"
   height="510.99362"
   id="svg3131"
   version="1.1"
   inkscape:version="0.48.4 r9939"
   sodipodi:docname="cforest.svg">
  <defs
     id="defs3133">
    <marker
       inkscape:stockid="Arrow2Lend"
       orient="auto"
       refY="0"
       refX="0"
       id="Arrow2Lend"
       style="overflow:visible">
      <path
         id="path4074"
         style="fill-rule:evenodd;stroke-width:0.625;stroke-linejoin:round"
         d="M 8.7185878,4.0337352 -2.2072895,0.01601326 8.7185884,-4.0017078 c -1.7454984,2.3720609 -1.7354408,5.6174519 -6e-7,8.035443 z"
         transform="matrix(-1.1,0,0,-1.1,-1.1,0)"
         inkscape:connector-curvature="0" />
    </marker>
    <inkscape:path-effect
       effect="spiro"
       id="path-effect4047"
       is_visible="true" />
    <marker
       inkscape:stockid="Arrow2Lend"
       orient="auto"
       refY="0"
       refX="0"
       id="Arrow2Lend-6"
       style="overflow:visible">
      <path
         inkscape:connector-curvature="0"
         id="path4074-0"
         style="fill-rule:evenodd;stroke-width:0.625;stroke-linejoin:round"
         d="M 8.7185878,4.0337352 -2.2072895,0.01601326 8.7185884,-4.0017078 c -1.7454984,2.3720609 -1.7354408,5.6174519 -6e-7,8.035443 z"
         transform="matrix(-1.1,0,0,-1.1,-1.1,0)" />
    </marker>
    <inkscape:path-effect
       effect="spiro"
       id="path-effect4047-2-8-0"
       is_visible="true" />
    <marker
       inkscape:stockid="Arrow2Lend"
       orient="auto"
       refY="0"
       refX="0"
       id="Arrow2Lend-1"
       style="overflow:visible">
      <path
         id="path4074-4"
         style="fill-rule:evenodd;stroke-width:0.625;stroke-linejoin:round"
         d="M 8.7185878,4.0337352 -2.2072895,0.01601326 8.7185884,-4.0017078 c -1.7454984,2.3720609 -1.7354408,5.6174519 -6e-7,8.035443 z"
         transform="matrix(-1.1,0,0,-1.1,-1.1,0)"
         inkscape:connector-curvature="0" />
    </marker>
    <inkscape:path-effect
       effect="spiro"
       id="path-effect4047-6"
       is_visible="true" />
    <marker
       inkscape:stockid="Arrow2Lend"
       orient="auto"
       refY="0"
       refX="0"
       id="Arrow2Lend-2"
       style="overflow:visible">
      <path
         id="path4074-3"
         style="fill-rule:evenodd;stroke-width:0.625;stroke-linejoin:round"
         d="M 8.7185878,4.0337352 -2.2072895,0.01601326 8.7185884,-4.0017078 c -1.7454984,2.3720609 -1.7354408,5.6174519 -6e-7,8.035443 z"
         transform="matrix(-1.1,0,0,-1.1,-1.1,0)"
         inkscape:connector-curvature="0" />
    </marker>
    <inkscape:path-effect
       effect="spiro"
       id="path-effect4047-6-2"
       is_visible="true" />
    <marker
       inkscape:stockid="Arrow2Lend"
       orient="auto"
       refY="0"
       refX="0"
       id="Arrow2Lend-6-1"
       style="overflow:visible">
      <path
         inkscape:connector-curvature="0"
         id="path4074-0-6"
         style="fill-rule:evenodd;stroke-width:0.625;stroke-linejoin:round"
         d="M 8.7185878,4.0337352 -2.2072895,0.01601326 8.7185884,-4.0017078 c -1.7454984,2.3720609 -1.7354408,5.6174519 -6e-7,8.035443 z"
         transform="matrix(-1.1,0,0,-1.1,-1.1,0)" />
    </marker>
    <inkscape:path-effect
       effect="spiro"
       id="path-effect4047-2-4"
       is_visible="true" />
    <marker
       inkscape:stockid="Arrow2Lend"
       orient="auto"
       refY="0"
       refX="0"
       id="marker3699"
       style="overflow:visible">
      <path
         inkscape:connector-curvature="0"
         id="path3701"
         style="fill-rule:evenodd;stroke-width:0.625;stroke-linejoin:round"
         d="M 8.7185878,4.0337352 -2.2072895,0.01601326 8.7185884,-4.0017078 c -1.7454984,2.3720609 -1.7354408,5.6174519 -6e-7,8.035443 z"
         transform="matrix(-1.1,0,0,-1.1,-1.1,0)" />
    </marker>
    <inkscape:path-effect
       effect="spiro"
       id="path-effect4047-2"
       is_visible="true" />
    <marker
       inkscape:stockid="Arrow2Lend"
       orient="auto"
       refY="0"
       refX="0"
       id="marker3704"
       style="overflow:visible">
      <path
         inkscape:connector-curvature="0"
         id="path3706"
         style="fill-rule:evenodd;stroke-width:0.625;stroke-linejoin:round"
         d="M 8.7185878,4.0337352 -2.2072895,0.01601326 8.7185884,-4.0017078 c -1.7454984,2.3720609 -1.7354408,5.6174519 -6e-7,8.035443 z"
         transform="matrix(-1.1,0,0,-1.1,-1.1,0)" />
    </marker>
    <inkscape:path-effect
       effect="spiro"
       id="path-effect4047-2-8"
       is_visible="true" />
  </defs>
  <sodipodi:namedview
     id="base"
     pagecolor="#ffffff"
     bordercolor="#666666"
     borderopacity="1.0"
     inkscape:pageopacity="0.0"
     inkscape:pageshadow="2"
     inkscape:zoom="1.4"
     inkscape:cx="285.64478"
     inkscape:cy="317.61134"
     inkscape:document-units="px"
     inkscape:current-layer="layer1"
     showgrid="false"
     fit-margin-top="0"
     fit-margin-left="0"
     fit-margin-right="0"
     fit-margin-bottom="0"
     inkscape:window-width="959"
     inkscape:window-height="927"
     inkscape:window-x="960"
     inkscape:window-y="27"
     inkscape:window-maximized="0" />
  <metadata
     id="metadata3136">
    <rdf:RDF>
      <cc:Work
         rdf:about="">
        <dc:format>image/svg+xml</dc:format>
        <dc:type
           rdf:resource="http://purl.org/dc/dcmitype/StillImage" />
        <dc:title />
      </cc:Work>
    </rdf:RDF>
  </metadata>
  <g
     inkscape:label="Capa 1"
     inkscape:groupmode="layer"
     id="layer1"
     transform="translate(-35.71875,-299.20807)">
    <g
       id="g3945">
      <rect
         style="fill:#f2f2f2;stroke:#645d61;stroke-width:1.27916777;stroke-miterlimit:10;stroke-dasharray:5.11667039, 5.11667039"
         x="138.20872"
         y="308.71606"
         stroke-miterlimit="10"
         width="313.72614"
         height="305.86368"
         id="rect291-3" />
      <g
         transform="translate(-214.86663,571.91222)"
         id="g3890">
        <text
           transform="matrix(0,-1,1,0,0,0)"
           id="text293-1-9"
           font-size="10"
           style="font-size:10px;fill:#645d61;font-family:Helvetica-Oblique"
           x="-32.920742"
           y="520.90527">StateSpace</text>
        <text
           transform="matrix(0,-1,1,0,0,0)"
           id="text293-1-9-1"
           font-size="10"
           style="font-size:10px;fill:#645d61;font-family:Helvetica-Oblique"
           x="-32.813564"
           y="594.49988">ProblemDefinition</text>
        <path
           sodipodi:nodetypes="cc"
           inkscape:connector-curvature="0"
           inkscape:original-d="m 524.94772,54.342432 0,-88.24965"
           inkscape:path-effect="#path-effect4047-2"
           id="path4045-8"
           d="m 524.94772,54.342432 0,-88.24965"
           style="fill:none;stroke:#000000;stroke-width:1;stroke-linecap:butt;stroke-linejoin:miter;stroke-miterlimit:4;stroke-opacity:1;stroke-dasharray:1, 2;stroke-dashoffset:0;marker-end:url(#Arrow2Lend-6)" />
        <path
           sodipodi:nodetypes="cc"
           inkscape:connector-curvature="0"
           inkscape:original-d="m 525.05157,-118.61003 0,-40.65864"
           inkscape:path-effect="#path-effect4047-2-4"
           id="path4045-8-6"
           d="m 525.05157,-118.61003 0,-40.65864"
           style="fill:none;stroke:#000000;stroke-width:1;stroke-linecap:butt;stroke-linejoin:miter;stroke-miterlimit:4;stroke-opacity:1;stroke-dasharray:1, 2;stroke-dashoffset:0;marker-end:url(#Arrow2Lend-6)" />
        <path
           sodipodi:nodetypes="cc"
           inkscape:connector-curvature="0"
           inkscape:original-d="m 596.76378,54.504352 0.12627,-213.874642"
           inkscape:path-effect="#path-effect4047-2-8"
           id="path4045-8-2"
           d="m 596.76378,54.504352 0.12627,-213.874642"
           style="fill:none;stroke:#000000;stroke-width:1;stroke-linecap:butt;stroke-linejoin:miter;stroke-miterlimit:4;stroke-opacity:1;stroke-dasharray:1, 2;stroke-dashoffset:0;marker-end:url(#Arrow2Lend-6)" />
        <g
           id="g3280"
           transform="translate(139.05586,-535.36054)">
          <path
             style="fill:#c0dcf3;stroke:#7992a2;stroke-width:1.58950901;stroke-miterlimit:10;stroke-dasharray:3.19173406, 3.19173406"
             inkscape:connector-curvature="0"
             stroke-miterlimit="10"
             d="m 523.8019,354.11333 c 0,11.54196 -10.19736,20.90096 -22.77044,20.90096 H 341.63835 c -12.57687,0 -22.77044,-9.359 -22.77044,-20.90096 v -52.24741 c 0,-11.54196 10.19357,-20.90095 22.77044,-20.90095 h 159.39311 c 12.57308,0 22.77044,9.35899 22.77044,20.90095 v 52.24741 z"
             id="path471-9-8" />
          <text
             sodipodi:linespacing="125%"
             id="text3945-1-0-6"
             y="277.68723"
             x="420.62238"
             style="font-size:14px;font-style:normal;font-weight:normal;text-align:center;line-height:125%;letter-spacing:0px;word-spacing:0px;text-anchor:middle;fill:#000000;fill-opacity:1;stroke:none;font-family:Sans"
             xml:space="preserve"><tspan
               id="tspan6226"
               style="fill:#000000"
               y="277.68723"
               x="420.62238"
               sodipodi:role="line" /><tspan
               style="fill:#000000;font-weight:bold;-inkscape-font-specification:Sans Bold"
               y="295.18723"
               x="420.62238"
               sodipodi:role="line"
               id="tspan3097">planner_t</tspan><tspan
               style="fill:#000000"
               y="312.68723"
               x="422.85089"
               sodipodi:role="line"
               id="tspan3099">Underlying planner instance </tspan><tspan
               id="tspan6252"
               style="fill:#000000"
               y="330.18723"
               x="422.85089"
               sodipodi:role="line">initialized with a new </tspan><tspan
               id="tspan6254"
               style="fill:#000000"
               y="347.68723"
               x="420.62238"
               sodipodi:role="line">SpaceInformation containing</tspan><tspan
               id="tspan6274"
               style="fill:#000000"
               y="365.18723"
               x="420.62238"
               sodipodi:role="line">CForestStateSpaceWrapper.</tspan></text>
        </g>
      </g>
      <text
         y="306.48834"
         x="151.19196"
         style="font-size:10px;fill:#645d61;font-family:Helvetica-Oblique"
         font-size="10"
         id="text293-1-3">Done for every planner instance</text>
    </g>
    <a
       xlink:href="classompl_1_1base_1_1CForestStateSpaceWrapper.html"
       id="a6589"
       transform="translate(-72.87485,34.756384)">
      <g
         id="g6578">
        <path
           style="fill:#c0dcf3;stroke:#7992a2;stroke-width:1.55922091;stroke-miterlimit:10;stroke-dasharray:3.13091554, 3.13091554"
           inkscape:connector-curvature="0"
           stroke-miterlimit="10"
           d="m 442.03478,482.92457 c 0,10.07402 -11.24227,18.24272 -25.10369,18.24272 H 241.20532 c -13.86561,0 -25.10369,-8.1687 -25.10369,-18.24272 v -45.60244 c 0,-10.07403 11.23808,-18.24272 25.10369,-18.24272 h 175.72577 c 13.86142,0 25.10369,8.16869 25.10369,18.24272 v 45.60244 z"
           id="path471-9" />
        <text
           sodipodi:linespacing="125%"
           id="text3945-1-0"
           y="421.6424"
           x="329.14423"
           style="font-size:14px;font-style:normal;font-weight:normal;text-align:center;line-height:125%;letter-spacing:0px;word-spacing:0px;text-anchor:middle;fill:#000000;fill-opacity:1;stroke:none;font-family:Sans"
           xml:space="preserve"><tspan
             style="font-weight:bold;fill:#000000;-inkscape-font-specification:Sans Bold"
             y="421.6424"
             x="329.14423"
             sodipodi:role="line"
             id="tspan3101" /><tspan
             style="font-weight:bold;fill:#000000;-inkscape-font-specification:Sans Bold"
             y="439.1424"
             x="329.14423"
             sodipodi:role="line"
             id="tspan3104">CForestStateSpaceWrapper</tspan><tspan
             style="fill:#000000"
             y="456.6424"
             x="331.37274"
             sodipodi:role="line"
             id="tspan3108">Encapsulates the StateSpace </tspan><tspan
             style="fill:#000000"
             y="474.1424"
             x="329.14423"
             sodipodi:role="line"
             id="tspan6033">provided by the user and adds</tspan><tspan
             style="fill:#000000"
             y="491.6424"
             x="329.14423"
             sodipodi:role="line"
             id="tspan6027"> specific CForest funcionalities.</tspan></text>
      </g>
    </a>
    <g
       id="g3833">
      <path
         style="fill:none;stroke:#000000;stroke-width:1;stroke-linecap:butt;stroke-linejoin:miter;stroke-miterlimit:4;stroke-opacity:1;stroke-dasharray:none;marker-end:url(#Arrow2Lend)"
         d="m 216.06488,662.13527 51.95415,0"
         id="path4045-0"
         inkscape:path-effect="#path-effect4047-6"
         inkscape:original-d="m 216.06488,662.13527 51.95415,0"
         inkscape:connector-curvature="0"
         sodipodi:nodetypes="cc" />
      <path
         style="fill:none;stroke:#000000;stroke-width:1;stroke-linecap:butt;stroke-linejoin:miter;stroke-miterlimit:4;stroke-opacity:1;stroke-dasharray:none;marker-end:url(#Arrow2Lend)"
         d="m 215.80627,712.57718 53.38272,-35.37509"
         id="path4045-0-2"
         inkscape:path-effect="#path-effect4047-6-2"
         inkscape:original-d="m 215.80627,712.57718 53.38272,-35.37509"
         inkscape:connector-curvature="0"
         sodipodi:nodetypes="cc" />
      <text
         id="text293-1"
         font-size="10"
         style="font-size:10px;fill:#645d61;font-family:Helvetica-Oblique"
         x="35.383671"
         y="623.87415">CForest inputs</text>
      <rect
         style="fill:#f2f2f2;stroke:#645d61;stroke-width:0.57643527;stroke-miterlimit:10;stroke-dasharray:2.30574113, 2.30574113"
         x="36.000965"
         y="626.73047"
         stroke-miterlimit="10"
         width="180.75441"
         height="107.80427"
         id="rect291-7" />
    </g>
    <g
       id="g6531"
       transform="translate(-103.40063,166.05897)">
      <g
         id="g5"
         transform="translate(361.37171,33.904407)">
        <rect
           id="rect7"
           height="45"
           width="108"
           y="450"
           x="225"
           style="fill:#645d61" />
        <text
           id="text9"
           font-size="14"
           transform="translate(244.7773,477)"
           style="font-size:14px;fill:#ffffff;font-family:Helvetica-Bold">User code</text>
      </g>
      <g
         id="g425-1"
         transform="matrix(0,-1,1,0,392.64069,892.72562)">
        <path
           sodipodi:nodetypes="cc"
           inkscape:connector-curvature="0"
           inkscape:original-d="m 386.29352,143.63309 0,48.73986"
           inkscape:path-effect="#path-effect4047"
           id="path4045"
           d="m 386.29352,143.63309 0,48.73986"
           style="fill:none;stroke:#000000;stroke-width:1;stroke-linecap:butt;stroke-linejoin:miter;stroke-miterlimit:4;stroke-opacity:1;stroke-dasharray:none;marker-end:url(#Arrow2Lend)" />
      </g>
    </g>
    <a
       xlink:href="classompl_1_1geometric_1_1CForest.html"
       transform="translate(-103.40063,165.8804)"
       id="a3200">
      <g
         id="g3189">
        <path
           id="path207-4"
           d="m 535.42954,531.48179 c 0,11.046 -8.059,20 -18,20 H 391.42955 c -9.941,0 -18,-8.954 -18,-20 v -50 c 0,-11.045 8.059,-20 18,-20 h 125.99999 c 9.941,0 18,8.955 18,20 v 50 z"
           stroke-miterlimit="10"
           inkscape:connector-curvature="0"
           style="fill:#7992a2;stroke:#645d61;stroke-width:2;stroke-miterlimit:10" />
        <text
           sodipodi:linespacing="125%"
           id="text3945"
           y="483.48181"
           x="454.1283"
           style="font-size:14px;font-style:normal;font-weight:normal;text-align:center;line-height:125%;letter-spacing:0px;word-spacing:0px;text-anchor:middle;fill:#000000;fill-opacity:1;stroke:none;font-family:Sans"
           xml:space="preserve"><tspan
             style="font-weight:bold;fill:#ffffff;stroke:none;-inkscape-font-specification:Sans Bold"
             y="483.48181"
             x="454.1283"
             id="tspan3947"
             sodipodi:role="line">CForest</tspan><tspan
             style="fill:#ffffff"
             y="500.98181"
             x="456.35681"
             sodipodi:role="line"
             id="tspan3136">Implements the </tspan><tspan
             style="fill:#ffffff"
             id="tspan3949"
             y="518.48181"
             x="456.35681"
             sodipodi:role="line">CForest meta-planner </tspan><tspan
             style="fill:#ffffff"
             id="tspan3951"
             y="535.98181"
             x="454.1283"
             sodipodi:role="line">as a regular planner.</tspan></text>
      </g>
    </a>
    <g
       id="g3502">
      <g
         id="g243"
         transform="translate(-128.61375,281.86769)">
        <rect
           id="rect245"
           height="78.334"
           width="322"
           y="450"
           x="396"
           style="fill:#f1f2f2" />
        <path
           id="path247"
           d="m 433,462.439 c 0,-1.131 -0.896,-2.049 -2,-2.049 h -14 c -1.104,0 -2,0.918 -2,2.049 v 5.121 c 0,1.131 0.896,2.049 2,2.049 h 14 c 1.104,0 2,-0.918 2,-2.049 v -5.121 z"
           stroke-miterlimit="10"
           inkscape:connector-curvature="0"
           style="fill:#7992a2;stroke:#645d61;stroke-width:2;stroke-miterlimit:10" />
        <path
           id="path249"
           d="m 433,485 c 0,1.104 -0.896,2 -2,2 h -14 c -1.104,0 -2,-0.896 -2,-2 v -5 c 0,-1.104 0.896,-2 2,-2 h 14 c 1.104,0 2,0.896 2,2 v 5 z"
           stroke-miterlimit="10"
           inkscape:connector-curvature="0"
           style="fill:#98bfd5;stroke:#645d61;stroke-miterlimit:10;stroke-dasharray:7.908, 3.954" />
        <path
           id="path251"
           d="m 433,501 c 0,1.104 -0.896,2 -2,2 h -14 c -1.104,0 -2,-0.896 -2,-2 v -5 c 0,-1.104 0.896,-2 2,-2 h 14 c 1.104,0 2,0.896 2,2 v 5 z"
           stroke-miterlimit="10"
           inkscape:connector-curvature="0"
           style="fill:#c0dcf3;stroke:#7992a2;stroke-miterlimit:10;stroke-dasharray:2.009, 2.009" />
        <text
           y="521.92828"
           x="414.21445"
           id="text253"
           font-size="10"
           style="font-size:10px;font-family:Helvetica-Oblique">A</text>
        <text
           y="521.92828"
           x="440.21445"
           id="text255"
           font-size="10"
           style="font-size:10px;font-family:Helvetica-Oblique">B</text>
        <text
           id="text257"
           transform="translate(442.001,469)">
          <tspan
             id="tspan259"
             font-size="10"
             y="0"
             x="0"
             style="font-size:10px;font-family:Helvetica">User must instantiate this class.</tspan>
          <tspan
             id="tspan261"
             font-size="10"
             y="17"
             x="0"
             style="font-size:10px;font-family:Helvetica">User must instantiate this class unless SimpleSetup is used.</tspan>
          <tspan
             id="tspan263"
             font-size="10"
             y="34"
             x="0"
             style="font-size:10px;font-family:Helvetica">CForest creates and configures this class.</tspan>
        </text>
        <g
           transform="translate(13.214288,3.9285718)"
           id="g265">
          <line
             id="line267"
             y2="514"
             x2="419.939"
             y1="514"
             x1="409"
             stroke-miterlimit="10"
             style="fill:none;stroke:#000000;stroke-miterlimit:10" />
          <g
             id="g269">
            <polygon
               id="polygon271"
               points="417.051,518.065 419.412,514 417.051,509.936 427,514 " />
          </g>
        </g>
      </g>
      <path
         style="fill:none;stroke:#000000;stroke-width:1;stroke-linecap:butt;stroke-linejoin:miter;stroke-miterlimit:4;stroke-opacity:1;stroke-dasharray:1, 2;stroke-dashoffset:0;marker-end:url(#Arrow2Lend-6)"
         d="m 335.43178,802.71847 64.37208,0.12627"
         id="path4045-8-2-9"
         inkscape:path-effect="#path-effect4047-2-8-0"
         inkscape:original-d="m 335.43178,802.71847 64.37208,0.12627"
         inkscape:connector-curvature="0"
         sodipodi:nodetypes="cc" />
      <text
         y="798.22095"
         x="338.48691"
         style="font-size:10px;fill:#645d61;font-family:Helvetica-Oblique"
         font-size="10"
         id="text293-1-4">Parameter</text>
    </g>
    <a
       xlink:href="classompl_1_1base_1_1SpaceInformation.html"
       id="a3097">
      <g
         transform="translate(114.39849,234.3212)"
         id="g5261">
        <a
           xlink:href="classompl_1_1base_1_1SpaceInformation.html"
           transform="matrix(1,0,0,0.81898239,-267.14295,235.15117)"
           id="a131-1">
          <g
             id="g133-4">
            <path
               style="fill:#99bfd5;stroke:#645d61;stroke-miterlimit:10;stroke-dasharray:7.908, 3.954"
               inkscape:connector-curvature="0"
               stroke-miterlimit="10"
               d="m 360,257 c 0,8.837 -8.06,16 -18,16 H 216 c -9.941,0 -18,-7.163 -18,-16 v -40 c 0,-8.837 8.059,-16 18,-16 h 126 c 9.94,0 18,7.163 18,16 v 40 z"
               id="path135-2" />
            <rect
               style="fill:none"
               x="206.666"
               y="207.16701"
               width="144.66701"
               height="59.667"
               id="rect137-3" />
          </g>
        </a>
        <text
           sodipodi:linespacing="125%"
           id="text3945-1"
           y="416.39478"
           x="11.419662"
           style="font-size:14px;font-style:normal;font-weight:normal;text-align:center;line-height:125%;letter-spacing:0px;word-spacing:0px;text-anchor:middle;fill:#000000;fill-opacity:1;stroke:none;font-family:Sans;-inkscape-font-specification:Sans"
           xml:space="preserve"><tspan
             style="font-weight:bold;fill:#000000;-inkscape-font-specification:Sans Bold"
             id="tspan3951-3"
             y="416.39478"
             x="11.419662"
             sodipodi:role="line">SpaceInformation</tspan><tspan
             style="font-weight:normal;fill:#000000;-inkscape-font-specification:Sans"
             y="433.89478"
             x="11.419662"
             sodipodi:role="line"
             id="tspan3103">Provides StateSpace</tspan><tspan
             style="font-weight:normal;fill:#000000;-inkscape-font-specification:Sans"
             y="451.39478"
             x="11.419662"
             sodipodi:role="line"
             id="tspan5063">and StateSampler</tspan></text>
      </g>
    </a>
    <a
       xlink:href="classompl_1_1base_1_1ProblemDefinition.html"
       id="a3148">
      <g
         id="g3138">
        <a
           xlink:href="classompl_1_1base_1_1ProblemDefinition.html"
           id="a6567"
           transform="translate(-104.45866,172.45902)">
          <g
             transform="translate(108.57143,146.07143)"
             id="g5274">
            <a
               xlink:href="classompl_1_1base_1_1ProblemDefinition.html"
               transform="matrix(1,0,0,0.33535831,-157.07143,316.56174)"
               id="a131-1-4">
              <g
                 id="g133-4-1">
                <path
                   style="fill:#99bfd5;stroke:#645d61;stroke-miterlimit:10;stroke-dasharray:7.908, 3.954"
                   inkscape:connector-curvature="0"
                   stroke-miterlimit="10"
                   d="m 360,257 c 0,8.837 -8.06,16 -18,16 H 216 c -9.941,0 -18,-7.163 -18,-16 v -40 c 0,-8.837 8.059,-16 18,-16 h 126 c 9.94,0 18,7.163 18,16 v 40 z"
                   id="path135-2-1" />
                <rect
                   style="fill:none"
                   x="206.666"
                   y="207.16701"
                   width="144.66701"
                   height="59.667"
                   id="rect137-3-3" />
              </g>
            </a>
          </g>
        </a>
        <text
           sodipodi:linespacing="125%"
           id="text3945-1-5"
           y="717.77209"
           x="128.33832"
           style="font-size:14px;font-style:normal;font-weight:normal;text-align:center;line-height:125%;letter-spacing:0px;word-spacing:0px;text-anchor:middle;fill:#000000;fill-opacity:1;stroke:none;font-family:Sans;-inkscape-font-specification:Sans"
           xml:space="preserve"><tspan
             style="font-weight:bold;fill:#000000;-inkscape-font-specification:Sans Bold"
             y="717.77209"
             x="128.33832"
             sodipodi:role="line"
             id="tspan5063-67">ProblemDefinition</tspan></text>
      </g>
    </a>
  </g>
</svg>

\endhtmlonly

Whenever a planner calls `SpaceInformation::allocStateSampler()` (or `allocDefaultStateSampler()`), `CForestStateSpaceWrapper` allocates a `CForestStateSampler` which creates and wraps the provided `StateSampler` (or default if none was specified).

\htmlonly
<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<!-- Created with Inkscape (http://www.inkscape.org/) -->

<svg
   xmlns:dc="http://purl.org/dc/elements/1.1/"
   xmlns:cc="http://creativecommons.org/ns#"
   xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"
   xmlns:svg="http://www.w3.org/2000/svg"
   xmlns="http://www.w3.org/2000/svg"
   xmlns:xlink="http://www.w3.org/1999/xlink"
   xmlns:sodipodi="http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd"
   xmlns:inkscape="http://www.inkscape.org/namespaces/inkscape"
   width="582.8631"
   height="254.422"
   id="svg3087"
   version="1.1"
   inkscape:version="0.48.4 r9939"
   sodipodi:docname="cforest_sampler.svg">
  <defs
     id="defs3089">
    <marker
       inkscape:stockid="Arrow2Lend"
       orient="auto"
       refY="0"
       refX="0"
       id="Arrow2Lend-6"
       style="overflow:visible">
      <path
         inkscape:connector-curvature="0"
         id="path4074-0"
         style="fill-rule:evenodd;stroke-width:0.625;stroke-linejoin:round"
         d="M 8.7185878,4.0337352 -2.2072895,0.01601326 8.7185884,-4.0017078 c -1.7454984,2.3720609 -1.7354408,5.6174519 -6e-7,8.035443 z"
         transform="matrix(-1.1,0,0,-1.1,-1.1,0)" />
    </marker>
    <inkscape:path-effect
       effect="spiro"
       id="path-effect4047-2-4-1"
       is_visible="true" />
    <marker
       inkscape:stockid="Arrow2Lend"
       orient="auto"
       refY="0"
       refX="0"
       id="Arrow2Lend-6-1"
       style="overflow:visible">
      <path
         inkscape:connector-curvature="0"
         id="path4074-0-7"
         style="fill-rule:evenodd;stroke-width:0.625;stroke-linejoin:round"
         d="M 8.7185878,4.0337352 -2.2072895,0.01601326 8.7185884,-4.0017078 c -1.7454984,2.3720609 -1.7354408,5.6174519 -6e-7,8.035443 z"
         transform="matrix(-1.1,0,0,-1.1,-1.1,0)" />
    </marker>
    <inkscape:path-effect
       effect="spiro"
       id="path-effect4047-2-6"
       is_visible="true" />
    <marker
       inkscape:stockid="Arrow2Lend"
       orient="auto"
       refY="0"
       refX="0"
       id="marker3200"
       style="overflow:visible">
      <path
         inkscape:connector-curvature="0"
         id="path3202"
         style="fill-rule:evenodd;stroke-width:0.625;stroke-linejoin:round"
         d="M 8.7185878,4.0337352 -2.2072895,0.01601326 8.7185884,-4.0017078 c -1.7454984,2.3720609 -1.7354408,5.6174519 -6e-7,8.035443 z"
         transform="matrix(-1.1,0,0,-1.1,-1.1,0)" />
    </marker>
    <inkscape:path-effect
       effect="spiro"
       id="path-effect4047-2-6-3"
       is_visible="true" />
    <marker
       inkscape:stockid="Arrow2Lend"
       orient="auto"
       refY="0"
       refX="0"
       id="marker3205"
       style="overflow:visible">
      <path
         inkscape:connector-curvature="0"
         id="path3207"
         style="fill-rule:evenodd;stroke-width:0.625;stroke-linejoin:round"
         d="M 8.7185878,4.0337352 -2.2072895,0.01601326 8.7185884,-4.0017078 c -1.7454984,2.3720609 -1.7354408,5.6174519 -6e-7,8.035443 z"
         transform="matrix(-1.1,0,0,-1.1,-1.1,0)" />
    </marker>
    <inkscape:path-effect
       effect="spiro"
       id="path-effect4047-2-6-3-5"
       is_visible="true" />
  </defs>
  <sodipodi:namedview
     id="base"
     pagecolor="#ffffff"
     bordercolor="#666666"
     borderopacity="1.0"
     inkscape:pageopacity="0.0"
     inkscape:pageshadow="2"
     inkscape:zoom="0.9899495"
     inkscape:cx="242.69185"
     inkscape:cy="35.866318"
     inkscape:document-units="px"
     inkscape:current-layer="g3165"
     showgrid="false"
     inkscape:window-width="1920"
     inkscape:window-height="927"
     inkscape:window-x="0"
     inkscape:window-y="27"
     inkscape:window-maximized="1"
     fit-margin-top="0"
     fit-margin-left="0"
     fit-margin-right="0"
     fit-margin-bottom="0" />
  <metadata
     id="metadata3092">
    <rdf:RDF>
      <cc:Work
         rdf:about="">
        <dc:format>image/svg+xml</dc:format>
        <dc:type
           rdf:resource="http://purl.org/dc/dcmitype/StillImage" />
        <dc:title />
      </cc:Work>
    </rdf:RDF>
  </metadata>
  <g
     inkscape:label="Capa 1"
     inkscape:groupmode="layer"
     id="layer1"
     transform="translate(-137.33253,-301.04945)">
    <g
       id="g3165"
       transform="translate(-189.90868,-70.710678)">
      <a
         xlink:href="classompl_1_1geometric_1_1CForest.html"
         id="a3561"
         transform="translate(341.36223,102)">
        <g
           id="g3550">
          <path
             id="path207-4-2"
             d="m 182.59444,340.76013 c 0,11.046 -8.059,20 -18,20 H 38.594448 c -9.941,0 -18,-8.954 -18,-20 v -50 c 0,-11.045 8.059,-20 18,-20 H 164.59444 c 9.941,0 18,8.955 18,20 v 50 z"
             stroke-miterlimit="10"
             inkscape:connector-curvature="0"
             style="fill:#7992a2;stroke:#645d61;stroke-width:2;stroke-miterlimit:10" />
          <text
             sodipodi:linespacing="125%"
             id="text3945-13"
             y="292.76013"
             x="101.29311"
             style="font-size:14px;font-style:normal;font-weight:normal;text-align:center;line-height:125%;letter-spacing:0px;word-spacing:0px;text-anchor:middle;fill:#000000;fill-opacity:1;stroke:none;font-family:Sans"
             xml:space="preserve"><tspan
               style="font-weight:bold;fill:#ffffff;-inkscape-font-specification:Sans Bold"
               y="292.76013"
               x="101.29311"
               id="tspan3947-8"
               sodipodi:role="line">CForest</tspan><tspan
               style="fill:#ffffff"
               y="310.26013"
               x="103.52163"
               sodipodi:role="line"
               id="tspan3045">Implements the </tspan><tspan
               style="fill:#ffffff"
               id="tspan3949-7"
               y="327.76013"
               x="103.52163"
               sodipodi:role="line">CForest meta-planner </tspan><tspan
               style="fill:#ffffff"
               id="tspan3951-4"
               y="345.26013"
               x="101.29311"
               sodipodi:role="line">as a regular planner.</tspan></text>
        </g>
      </a>
      <g
         id="g3299">
        <g
           id="g3176">
          <g
             id="g3125"
             transform="translate(189.90868,70.710678)">
            <path
               id="path471-9-8-1"
               d="m 239.50267,517.38067 c 0,3.4796 -5.06859,6.3011 -11.31805,6.3011 h -79.22638 c -6.25134,0 -11.31806,-2.8215 -11.31806,-6.3011 v -15.75126 c 0,-3.47961 5.06672,-6.30111 11.31806,-6.30111 h 79.22638 c 6.24946,0 11.31805,2.8215 11.31805,6.30111 v 15.75126 z"
               stroke-miterlimit="10"
               inkscape:connector-curvature="0"
               style="fill:#c0dcf3;stroke:#7992a2;stroke-width:0.61530203;stroke-miterlimit:10;stroke-dasharray:1.23552649, 1.23552649" />
          </g>
          <path
             style="fill:none;stroke:#000000;stroke-width:1;stroke-linecap:butt;stroke-linejoin:miter;stroke-miterlimit:4;stroke-opacity:1;stroke-dasharray:1, 2;stroke-dashoffset:0;marker-end:url(#Arrow2Lend-6)"
             d="m 429.68524,580.26346 251.01578,0"
             id="path4045-8-6-4"
             inkscape:path-effect="#path-effect4047-2-4-1"
             inkscape:original-d="m 429.68524,580.26346 251.01578,0"
             inkscape:connector-curvature="0"
             sodipodi:nodetypes="cc" />
          <text
             y="566.81396"
             x="437.04309"
             style="font-size:10px;fill:#645d61;font-family:Helvetica-Oblique"
             font-size="10"
             id="text293-1-2">SpaceInformation::allocStateSampler()</text>
          <text
             y="576.77386"
             x="437.06424"
             style="font-size:10px;fill:#645d61;font-family:Helvetica-Oblique"
             font-size="10"
             id="text293-1-2-3">SpaceInformation::allocDefaultStateSampler()</text>
        </g>
        <text
           transform="matrix(0,-1,1,0,0,0)"
           id="text293-1-9-8"
           font-size="10"
           style="font-size:10px;fill:#645d61;font-family:Helvetica-Oblique"
           x="-529.94385"
           y="772.97223">StateSpace</text>
        <path
           sodipodi:nodetypes="cc"
           inkscape:connector-curvature="0"
           inkscape:original-d="m 775.87834,539.36552 0,-78.24965"
           inkscape:path-effect="#path-effect4047-2-6"
           id="path4045-8-5"
           d="m 775.87834,539.36552 0,-78.24965"
           style="fill:none;stroke:#000000;stroke-width:1;stroke-linecap:butt;stroke-linejoin:miter;stroke-miterlimit:4;stroke-opacity:1;stroke-dasharray:1, 2;stroke-dashoffset:0;marker-end:url(#Arrow2Lend-6)" />
        <path
           sodipodi:nodetypes="cc"
           inkscape:connector-curvature="0"
           inkscape:original-d="m 661.30743,417.9386 -134.89524,0"
           inkscape:path-effect="#path-effect4047-2-6-3"
           id="path4045-8-5-1"
           d="m 661.30743,417.9386 -134.89524,0"
           style="fill:none;stroke:#000000;stroke-width:1;stroke-linecap:butt;stroke-linejoin:miter;stroke-miterlimit:4;stroke-opacity:1;stroke-dasharray:1, 2;stroke-dashoffset:0;marker-end:url(#Arrow2Lend-6)" />
        <path
           sodipodi:nodetypes="cc"
           inkscape:connector-curvature="0"
           inkscape:original-d="M 661.53796,417.60602 390.07582,565.97605"
           inkscape:path-effect="#path-effect4047-2-6-3-5"
           id="path4045-8-5-1-0"
           d="M 661.53796,417.60602 390.07582,565.97605"
           style="fill:none;stroke:#000000;stroke-width:1;stroke-linecap:butt;stroke-linejoin:miter;stroke-miterlimit:4;stroke-opacity:1;stroke-dasharray:1, 2;stroke-dashoffset:0;marker-end:url(#Arrow2Lend-6)" />
        <text
           y="414.49271"
           x="539.48126"
           style="font-size:10px;fill:#645d61;font-family:Helvetica-Oblique"
           font-size="10"
           id="text293-1-2-9">CForest::addSampler()</text>
        <text
           transform="matrix(0.8660254,-0.5,0.5,0.8660254,0,0)"
           y="683.83313"
           x="175.83974"
           style="font-size:10px;fill:#645d61;font-family:Helvetica-Oblique"
           font-size="10"
           id="text293-1-2-9-2">return sampler</text>
      </g>
      <a
         xlink:href="classompl_1_1base_1_1CForestStateSpaceWrapper.html"
         transform="translate(448.60535,119.94952)"
         id="a6589-1">
        <g
           id="g6578-7">
          <path
             style="fill:#c0dcf3;stroke:#7992a2;stroke-width:1.60128808;stroke-miterlimit:10;stroke-dasharray:3.21538636, 3.21538636"
             inkscape:connector-curvature="0"
             stroke-miterlimit="10"
             d="m 460.69829,486.24617 c 0,10.59481 -11.27423,19.1858 -25.17507,19.1858 H 259.29774 c -13.90503,0 -25.17507,-8.59099 -25.17507,-19.1858 v -47.95993 c 0,-10.59481 11.27004,-19.1858 25.17507,-19.1858 h 176.22548 c 13.90084,0 25.17507,8.59099 25.17507,19.1858 v 47.95993 z"
             id="path471-9-4" />
          <text
             sodipodi:linespacing="125%"
             id="text3945-1-0-2"
             y="440.21381"
             x="347.14423"
             style="font-size:14px;font-style:normal;font-weight:normal;text-align:center;line-height:125%;letter-spacing:0px;word-spacing:0px;text-anchor:middle;fill:#000000;fill-opacity:1;stroke:none;font-family:Sans"
             xml:space="preserve"><tspan
               style="font-weight:bold;fill:#000000;-inkscape-font-specification:Sans Bold"
               y="440.21381"
               x="347.14423"
               sodipodi:role="line"
               id="tspan5063-6-4">CForestStateSpaceWrapper</tspan><tspan
               style="fill:#000000"
               y="457.71381"
               x="349.37274"
               sodipodi:role="line"
               id="tspan3092">Encapsulates the StateSpace </tspan><tspan
               style="fill:#000000"
               y="475.21381"
               x="347.14423"
               sodipodi:role="line"
               id="tspan6033-5">provided by the user and adds</tspan><tspan
               style="fill:#000000"
               y="492.71381"
               x="347.14423"
               sodipodi:role="line"
               id="tspan6027-5"> specific CForest funcionalities.</tspan></text>
        </g>
      </a>
      <a
         xlink:href="classompl_1_1base_1_1CForestStateSampler.html"
         transform="translate(418.21756,-42.209934)"
         id="a6589-7">
        <g
           id="g6578-6">
          <path
             id="path471-9-1"
             d="m 469.28869,483.47897 c 0,10.16117 -11.20504,18.40054 -25.02056,18.40054 H 269.12425 c -13.81968,0 -25.02055,-8.23937 -25.02055,-18.40054 v -45.99696 c 0,-10.16117 11.20087,-18.40054 25.02055,-18.40054 h 175.14388 c 13.81552,0 25.02056,8.23937 25.02056,18.40054 v 45.99696 z"
             stroke-miterlimit="10"
             inkscape:connector-curvature="0"
             style="fill:#c0dcf3;stroke:#7992a2;stroke-width:1.56335592;stroke-miterlimit:10;stroke-dasharray:3.13921874, 3.13921874" />
          <text
             xml:space="preserve"
             style="font-size:14px;font-style:normal;font-weight:normal;text-align:center;line-height:125%;letter-spacing:0px;word-spacing:0px;text-anchor:middle;fill:#000000;fill-opacity:1;stroke:none;font-family:Sans"
             x="355.85852"
             y="436.9281"
             id="text3945-1-0-9"
             sodipodi:linespacing="125%"><tspan
               id="tspan5063-6-5"
               sodipodi:role="line"
               x="355.85852"
               y="436.9281"
               style="font-weight:bold;fill:#000000;-inkscape-font-specification:Sans Bold">CForestStateSampler</tspan><tspan
               sodipodi:role="line"
               x="358.08704"
               y="454.4281"
               style="fill:#000000"
               id="tspan3086">Encapsulates the StateSampler </tspan><tspan
               id="tspan6027-3"
               sodipodi:role="line"
               x="358.08704"
               y="471.9281"
               style="fill:#000000">provided and adds specific </tspan><tspan
               sodipodi:role="line"
               x="355.85852"
               y="489.4281"
               style="fill:#000000"
               id="tspan3089">CForest funcionalities.</tspan></text>
        </g>
      </a>
      <text
         sodipodi:linespacing="125%"
         id="text3945-13-5"
         y="584.38568"
         x="377.84814"
         style="font-size:14px;font-style:normal;font-weight:normal;text-align:center;line-height:125%;letter-spacing:0px;word-spacing:0px;text-anchor:middle;fill:#000000;fill-opacity:1;stroke:none;font-family:Sans"
         xml:space="preserve"><tspan
           style="font-weight:bold;fill:#000000;-inkscape-font-specification:Sans Bold"
           id="tspan3951-4-7"
           y="584.38568"
           x="377.84814"
           sodipodi:role="line">planner_t</tspan></text>
    </g>
  </g>
</svg>

\endhtmlonly

Therefore, CForest tracks the creation of the planners but, thanks to the `CForestStateSpaceWrapper`, it also tracks the creation of the state samplers. This allows CForest to maintain a planner-sampler correspondence required to shared paths between trees.

This new state sampler allows given states (as opposed to random states) to be sampled. Therefore, when a new, better solution is found, CForest shares the states of the new solution to all samplers. `CForestStateSampler` will sample these states. Once states are sampled, it will continue sampling states according to its underlying state sampler.

## Limitations {#cf_limitations}

- CForest is designed to solve single-query, shortest path planning problems. Therefore, not all the ompl::base::OptimizationObjective instantiations are valid. The cost metric has to obey the triangle inequiality. It is important to note that shortest path planning does not mean that only the path length can be optimized. Other metrics could be specified: time, energy, etc. However, clearance or smoothness optimization are examples of non-valid optimization objetives.

- Whenever the tree is pruned, the states are removed from the `NearestNeighbors` data structure. However, current implementation does not remove pruned states until the planner instance is destroyed. This is specific for the underlying planner implementations but this is the most efficient way in terms of computation time.

## Make your planner CForest-compatible {#cf_compatible}

If you have implemented an __incremental, optimizing planner__ in OMPL and want make it complatible with CForest, there are few modifications you should carry out on your planner.

There are three main components that should be included:
- CForest activation and configuration.
- Path sharing.
- Tree pruning.


### CForest activation and configuration

Firstly, if working under CForest, the corresponding callback for sharing intermediate solutions must be set. Also, a check should be included to figure out if the pruning would cause some errors. Therefore, in the `solve()` function, you need to add the following code:

~~~{.cpp}
ompl::geoemtric::MyPlanner solve(const base::PlannerTerminationCondition &ptc)
{
    // Configuration before entering the main solving loop.
    ....
    if (prune_ && !si_->getStateSpace()->isMetricSpace())
       OMPL_WARN("%s: tree pruning was activated but since the state space %s is not a metric space, the optimization objective might not satisfy the triangle inequality. You may need to disable pruning.", getName().c_str(), si_->getStateSpace()->getName().c_str());

    const base::ReportIntermediateSolutionFn intermediateSolutionCallback = pdef_->getIntermediateSolutionCallback();
    ...
    while (ptc == false) // Main loop
    ...
}
~~~

`prune_` flag will be set but the user or the CForest framework. This flag manages the code related to tree pruning and early state rejection. If `intermediateSolutionCallback` is false, then the path sharing code will not be executed.

Since all CForest threads share the same `PlannerTerminationCondition`, it is recommended to include the following call once the main loop of the solve() function has finished:

~~~{.cpp}
ompl::geoemtric::MyPlanner solve(const base::PlannerTerminationCondition &ptc)
{
    while (ptc == false) // Main loop
    {
        ...
    }

    ptc.terminate(); // Will force other threads to stop.
}
~~~

This is particularly useful when one of the threads breaks out the loop but the ptc will still evaluate to true. For instance, this happens in RRTstar everytime a cost threshold is set.

\note CForest can be used wihtout pruning. In this case, the `prune_` flag is activated only if the ompl::geometricCForest::setPrune() method was called with a true argument (it is activated by default). Tree pruning in RRTstar can be used as an independent feature.

### Path sharing

As CForest is designed to use incremental, optimizing planners, it is assumed you will have a flag in your code to indicate when a new, better path has been found and a pointer to the motion which contains the last state (goal) of the solution. Therefore, at the end of the main loop within the solve function, you should add a code similar to the following:

~~~{.cpp}
ompl::geoemtric::MyPlanner solve(const base::PlannerTerminationCondition &ptc)
{
    // Configuration before entering the main solving loop.
    ...
    while (ptc == false) // Main loop
    {
        ...
        // Sampling new states.
        ...
        // Addind states to the tree.
        ...
        // Searching for improved solutions.
        ...
        if (updatedSolution)
        {
            ...
            if (intermediateSolutionCallback)
            {
                std::vector<const base::State *> spath;
                Motion *intermediate_solution = solution->parent;

                do
                {
                    spath.push_back(intermediate_solution->state);
                    intermediate_solution = intermediate_solution->parent;
                } while (intermediate_solution->parent != 0);

                intermediateSolutionCallback(this, spath, bestCost_);
            }
        } // if updated solution
    } // while ptc
}
~~~

\note The spath vector has to contain the states of the solution from the goal to the start (in this specific order).

In this case, the goal and start states are not being included since it is usually harder to deal with those two states. Code simplicity prevails, but this really depends on the underlying planner used together with CForest.

It is likely to share more than one path in which one or more states are modified. This would imply that the tree and solution path would have repeated states. To avoid this, add the following code within the `solve()` method just after sampling a new state:

~~~{.cpp}
ompl::geoemtric::MyPlanner solve(const base::PlannerTerminationCondition &ptc)
{
    // Configuration before entering the main solving loop.
    ...
    while (ptc == false) // Main loop
    {
        // Sample a new state and store it in rmotion.
        ...
        // find closest state in the tree
        Motion *nmotion = nn_->nearest(rmotion);
        if (intermediateSolutionCallback && si_->equalStates(nmotion->state, rstate))
            continue;
        ...
    }
}
~~~

If a sampled state is repeated it will be discarded and the next iteration of the main loop will start.

### Tree pruning

Prunning refers to two different ways of remove states: 1) prune those states already on the tree that do not lead to a better solution and 2) reject those states before adding them to the tree. In both cases, all the modifications are again within the `solve()` method. Tree main modifications are done:

1. **Early state rejection** Check if random samples can lead to a better solution (cost used: heuristic from start to state + heuristic from state to goal)
2. **State rejection**Most of the states satisfy 1., but once they are wired into the tree, they cannot lead to a better solution (cost used: cost to go from start to state + heuristic from state to goal)  (early state rejection).
3. **Tree pruning** If a new, better solution is found, the pruning threshold will be decreased, therefore prune the states of the tree so that all those states which higher cost than the current best cost are removed.

All these modifications are included in the following code example:

~~~{.cpp}
ompl::geometric::MyPlanner solve(const base::PlannerTerminationCondition &ptc)
{
    // Configuration before entering the main solving loop.
    ...
    while (ptc == false) // Main loop
    {
        ...
        // Sampling new states.
        ...
        sampler_->sampleUniform(rstate);

        if (prune) // Modification 1 - early state rejection
        {
            const base::Cost costTotal =  computeLowestCostToGo(rmotion);
            if (opt_->isCostBetterThan(bestCost_, costTotal))
                continue;
        }

        // Adding states to the tree.
        ...
        if (prune) // Modification 2 - state rejection.
        {
            const base::Cost costTotal = computeCostToGoal(motion);
            if (opt_->isCostBetterThan(costTotal, bestCost_))
            {
                nn_->add(motion);
                motion->parent->children.push_back(motion);
            }
            else // If the new motion does not improve the best cost it is ignored.
            {
                si_->freeState(motion->state);
                delete motion;
                continue;
            }
        }
        else // Regular code when prune is not activated.
        {
            // add motion to the tree
            nn_->add(motion);
            motion->parent->children.push_back(motion);
        }
        // Searching for improved solutions.
        ...
        if (updatedSolution)
        {
             ...
             if (prune_) // Modification 3 - tree pruning.
                 pruneTree(bestCost_);
             ...
        } // if updated solution
    } // while ptc
}
~~~

\note You should implement the `pruneTree()` function for your code. Most probably, the available RRTstar::pruneTree() method would be directly applicable. It is possible to use CForest without this method. However, it is highly recommended to at least include the modifications 1 & 2 about state rejection.

For a complete example of how to make these modifications, it is recommended to analyze the ompl::geometric::RRTstar::solve() method.
