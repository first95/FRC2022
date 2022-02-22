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
    List rgbThresholdGreen = [0.0, 90.6965394039314];
    List rgbThresholdBlue = [0.0, 97.31176957286172];

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
    Double filterContoursMinArea = 10.0;
    Double filterContoursMinPerimeter = 50.0;
    Double filterContoursMinWidth = 2.0;
    Double filterContoursMaxWidth = 200.0;
    Double filterContoursMinHeight = 2.0;
    Double filterContoursMaxHeight = 1000.0;
    List filterContoursSolidity = [57.5539615514467, 100];
    Double filterContoursMaxVertices = 1000000.0;
    Double filterContoursMinVertices = 0.0;
    Double filterContoursMinRatio = 1.0;
    Double filterContoursMaxRatio = 4.0;

    filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);
}




