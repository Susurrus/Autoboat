% Convert GPS from a sim run into a KML file for Google Earth to open.

indices = 2:100:length(globalPosition);
waypointPlot = ge_plot(globalPosition(indices, 2), ...
                       globalPosition(indices, 1));
                
%pointCloud = ge_point(globalPosition(indices, 2), globalPosition(indices, 1), globalPosition(indices, 3));
%waypointPlot = [ge_folder('Track', waypointPlot) ge_folder('Point', pointCloud)];
                   
%ge_output('testing.kml', waypointPlot);
ge_output('testing.kml', waypointPlot);