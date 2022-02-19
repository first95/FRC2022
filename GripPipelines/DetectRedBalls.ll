//
// Inputs
//
Inputs
{
	Mat source0;
}

//
// Variables
//
Outputs
{
	Mat rgbThresholdOutput;
	ContoursReport findContoursOutput;
	ContoursReport filterContoursOutput;
}

//
// Steps
//

Step RGB_Threshold0
{
    Mat rgbThresholdInput = source0;
    List rgbThresholdRed = [165.10789070626814, 255.0];
    List rgbThresholdGreen = [0.0, 167.96926667665866];
    List rgbThresholdBlue = [0.0, 181.02389078498294];

    rgbThreshold(rgbThresholdInput, rgbThresholdRed, rgbThresholdGreen, rgbThresholdBlue, rgbThresholdOutput);
}

Step Find_Contours0
{
    Mat findContoursInput = rgbThresholdOutput;
    Boolean findContoursExternalOnly = false;

    findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);
}

Step Filter_Contours0
{
    ContoursReport filterContoursContours = findContoursOutput;
    Double filterContoursMinArea = 1000.0;
    Double filterContoursMinPerimeter = 50.0;
    Double filterContoursMinWidth = 20.0;
    Double filterContoursMaxWidth = 200.0;
    Double filterContoursMinHeight = 20.0;
    Double filterContoursMaxHeight = 50.0;
    List filterContoursSolidity = [57.5539615514467, 100];
    Double filterContoursMaxVertices = 1000000.0;
    Double filterContoursMinVertices = 0.0;
    Double filterContoursMinRatio = 1.0;
    Double filterContoursMaxRatio = 3.0;

    filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);
}




