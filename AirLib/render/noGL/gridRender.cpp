#include <render/gridRender.h>
#include <utils/util.h>

Point3f greenCube(0.5,1.0,0.5);
Point3f redCube(1.0,0.5,0.5);
Point3f blueCube(0.5,0.5,1.0);
Point3f whiteCube(0.8,0.8,0.8);
Point3f brightWhiteCube(1,1,1);

void drawQuad(Point3f orig, Point3f direction1, Point3f direction2, Point3f normal, Point3f color, bool blend)
{
}

void drawCube(Point3d o, double cellSize, Point3f color,vector<bool>& renderSide, bool blend = false, bool lighting = false)
{
}

void drawPointLocator(Point3d pt, float size, bool spot)
{
}

void gridRenderer::drawFunc(object* obj)
{
 
}

bool gridRenderer::update(object* obj)
{
    if(!dirtyFlag && !dirtyXZ && !dirtyXY)
        return true;
    else
    {
        updateSlices();
        dirtyFlag = false;
    }

    return true;
}


void gridRenderer::init(int numLabels)
{
    int boneCount = numLabels;
    grid->valueRange = boneCount;
    grid->tradIds.resize(boneCount);
    for(int i = 0; i< boneCount; i++)
    {
        grid->tradIds[i] = rand()%boneCount;

        if(i > 0)
        {
            while(grid->tradIds[i] == grid->tradIds[i-1])
                grid->tradIds[i] = rand()%boneCount;
        }
    }
}

void gridRenderer::updateSlicesForSegmentation(int maxRange)
{
	XYValue = floor(((float)m_iCurrentSliceXY/1000.0)*((float)grid->dimensions.Z()));
	XZValue = floor(((double)(m_iCurrentSliceXZ)/1000.0)*((float)grid->dimensions.Y()));

	if(dirtyXY)
	{
		for(unsigned int i = 0; i< sliceValuesXY.size(); i++)
		{
			for(unsigned int j = 0; j< sliceValuesXY[i].size(); j++)
			{
				if(grid->cells[i][j][XYValue]->getType() == EXTERIOR)
				{
					sliceValuesXY[i][j].X() = 1.0;
					sliceValuesXY[i][j].Y() = 1.0;
					sliceValuesXY[i][j].Z() = 1.0;
				}
				else
				{
					if(grid->cells[i][j][XYValue]->data->segmentId < 0)
					{
						sliceValuesXY[i][j].X() = 0.0;
						sliceValuesXY[i][j].Y() = 0.0;
						sliceValuesXY[i][j].Z() = 0.0;
					}
					else
					{
						//int idOwnerTraduced = tradIds[grid.cells[i][j][XYValue]->data->segmentId];
						float idOwnerTraduced = (grid->cells[i][j][XYValue]->data->segmentId * 13) % maxRange;
						GetColour(idOwnerTraduced/maxRange, 0.0,1.0,
							  sliceValuesXY[i][j].X(), sliceValuesXY[i][j].Y(), sliceValuesXY[i][j].Z());
					}
				}
			}
		}
		dirtyXY = false;
	}

	if(dirtyXZ)
	{
		for(unsigned int i = 0; i< sliceValuesXZ.size(); i++)
		{
			for(unsigned int j = 0; j< sliceValuesXZ[i].size(); j++)
			{
				if(grid->cells[i][XZValue][j]->getType() == EXTERIOR)
				{
					sliceValuesXZ[i][j].X() = 1.0;
					sliceValuesXZ[i][j].Y() = 1.0;
					sliceValuesXZ[i][j].Z() = 1.0;
				}
				else
				{
					if(grid->cells[i][XZValue][j]->data->ownerLabel<0)
					{
						sliceValuesXZ[i][j].X() = 0.0;
						sliceValuesXZ[i][j].Y() = 0.0;
						sliceValuesXZ[i][j].Z() = 0.0;
					}
					else
					{
						//int idOwnerTraduced = tradIds[grid.cells[i][XZValue][j]->data->ownerLabel];
						float idOwnerTraduced = (grid->cells[i][XZValue][j]->data->segmentId * 13) % maxRange;
						GetColour(idOwnerTraduced/maxRange, 0.0,1.0,
								  sliceValuesXZ[i][j].X(), sliceValuesXZ[i][j].Y(), sliceValuesXZ[i][j].Z());
					}
				}
			}
		}

		dirtyXZ = false;	
	}
}

void gridRenderer::updateSlicesForVolumeLabels(int maxRange)
{
	XYValue = floor(((float)m_iCurrentSliceXY/1000.0)*((float)grid->dimensions.Z()));
	XZValue = floor(((double)(m_iCurrentSliceXZ)/1000.0)*((float)grid->dimensions.Y()));

	assert(maxRange != 0);

	//printf("vis_labels.\n"); fflush(0);
	if(dirtyXY)
	{
	//if(sliceValuesXY.size() > 0)
		//printf("se va a actualizar\n"); fflush(0);

	for(unsigned int i = 0; i< sliceValuesXY.size(); i++)
	{
		for(unsigned int j = 0; j< sliceValuesXY[i].size(); j++)
		{
			if(grid->cells[i][j][XYValue]->getType() == BOUNDARY)
				sliceValuesXY[i][j] = Point3f(0.8,0.1,0.1);
			else if(grid->cells[i][j][XYValue]->getType() == INTERIOR)
				sliceValuesXY[i][j] = Point3f(0.1,0.8,0.1);
			else if(grid->cells[i][j][XYValue]->getType() == EXTERIOR)
				sliceValuesXY[i][j] = Point3f(0.1,0.1,0.8);
		}
	}
	dirtyXY = false;
	//printf("actualizado XY\n"); fflush(0);
	}

	if(dirtyXZ)
	{
	//if(sliceValuesXZ.size() > 0)
	   // printf("se va a actualizar\n"); fflush(0);

	for(unsigned int i = 0; i< sliceValuesXZ.size(); i++)
	{
		for(unsigned int j = 0; j< sliceValuesXZ[i].size(); j++)
		{
			if(grid->cells[i][XZValue][j]->getType() == BOUNDARY)
				sliceValuesXZ[i][j] = Point3f(0.8,0.1,0.1);
			else if(grid->cells[i][XZValue][j]->getType() == INTERIOR)
				sliceValuesXZ[i][j] = Point3f(0.1,0.8,0.1);
			else if(grid->cells[i][XZValue][j]->getType() == EXTERIOR)
				sliceValuesXZ[i][j] = Point3f(0.1,0.1,0.8);
		}
	}

	dirtyXZ = false;
	//printf("actualizado XZ\n"); fflush(0);
	}
}

void gridRenderer::updateSlicesForWeights(float maxRange, int desiredIndex)
{
	int searchedIndex = -1;

	XYValue = floor(((float)m_iCurrentSliceXY/1000.0)*((float)grid->dimensions.Z()));
	XZValue = floor(((double)(m_iCurrentSliceXZ)/1000.0)*((float)grid->dimensions.Y()));

	if(dirtyXY)
	{
		for(unsigned int i = 0; i< sliceValuesXY.size(); i++)
		{
			for(unsigned int j = 0; j< sliceValuesXY[i].size(); j++)
			{
				if(grid->cells[i][j][XYValue]->getType() == EXTERIOR)
				{
					sliceValuesXY[i][j].X() = 1.0;
					sliceValuesXY[i][j].Y() = 1.0;
					sliceValuesXY[i][j].Z() = 1.0;
				}

				else
				{
					//vector<int>* labvector = &(grid.cells[i][j][XYValue]->data->labels);
					//vector<float>* weightvector = &(grid.cells[i][j][XYValue]->data->weights);
					searchedIndex = -1;
					for(unsigned int ce = 0; ce < grid->cells[i][j][XYValue]->data->influences.size(); ce++)
					{
						if(grid->cells[i][j][XYValue]->data->influences[ce].label == desiredIndex)
						{
							searchedIndex = ce;
							break;
						}
					}

					if(searchedIndex >= 0)
						GetColour(grid->cells[i][j][XYValue]->data->influences[searchedIndex].weightValue, 0.0,1.0,
						  sliceValuesXY[i][j].X(), sliceValuesXY[i][j].Y(), sliceValuesXY[i][j].Z());
					else
					{
						GetColour(0.0,0.0,1.0, sliceValuesXY[i][j].X(), sliceValuesXY[i][j].Y(), sliceValuesXY[i][j].Z());
					}

				}
			}
		}
		dirtyXY = false;
	}

	if(dirtyXZ)
	{
		for(unsigned int i = 0; i< sliceValuesXZ.size(); i++)
		{
			for(unsigned int j = 0; j< sliceValuesXZ[i].size(); j++)
			{
				if(grid->cells[i][XZValue][j]->getType() == EXTERIOR)
				{
					sliceValuesXZ[i][j].X() = 1.0;
					sliceValuesXZ[i][j].Y() = 1.0;
					sliceValuesXZ[i][j].Z() = 1.0;
				}
				else
				{
					searchedIndex = -1;
					//vector<int>* labvector = &(grid.cells[i][XZValue][j]->data->labels);
					//vector<float>* weightvector = &(grid.cells[i][XZValue][j]->data->weights);
					for(unsigned int ce = 0; ce< grid->cells[i][XZValue][j]->data->influences.size(); ce++)
					{
						if(grid->cells[i][XZValue][j]->data->influences[ce].label == desiredIndex)
						{
							searchedIndex = ce;
							break;
						}
					}

					if(searchedIndex >= 0)
					{
						GetColour((float)grid->cells[i][XZValue][j]->data->influences[searchedIndex].weightValue, 0.0,1.0,
						  sliceValuesXZ[i][j].X(), sliceValuesXZ[i][j].Y(), sliceValuesXZ[i][j].Z());
					}
					else
					{
						GetColour(0, 0.0,1.0, sliceValuesXZ[i][j].X(), sliceValuesXZ[i][j].Y(), sliceValuesXZ[i][j].Z());
					}
				}
			}
		}

		dirtyXZ = false;
	}
}

void gridRenderer::updateSlices()
{

    if(grid->valueRange == 0)
    {
        printf("tenemos un problema con valueRange.\n"); fflush(0);
        return;
    }

    if(!Initialized)
    {
        int dimX = grid->dimensions.X();
        sliceValuesXY.clear(); sliceValuesXY.resize(dimX);
        sliceValuesXZ.resize(grid->dimensions.X());

        for(unsigned int i = 0; i< sliceValuesXY.size(); i++)
            sliceValuesXY[i].resize(grid->dimensions.Y());

        for(unsigned int i = 0; i< sliceValuesXZ.size(); i++)
            sliceValuesXZ[i].resize(grid->dimensions.Z());

        Initialized = true;
    }

    int maxRange = grid->valueRange;
    assert(maxRange != 0);

    int desiredIndex = 0;
    if(desiredVertex >= 0)  desiredIndex = desiredVertex;

    switch(iVisMode)
    {
    case VIS_LABELS:
    default:
		updateSlicesForVolumeLabels(maxRange);
    break;
    
	case VIS_WEIGHTS:
        updateSlicesForWeights(1.0, desiredIndex);
        break;

    case VIS_SEGMENTATION:
		updateSlicesForSegmentation(maxRange);
        break;
    }
}
