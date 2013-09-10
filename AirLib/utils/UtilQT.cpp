#include "util.h"
//#include <QDir>
//#include <QString>
//#include <QDateTime>
//#include <QTextStream>
//#include <QDataStream>

/*void mkDirRecursive(QString dirPath)
{

	QDir dir(dirPath);
    if(!dir.exists())
    {
        QStringList list = dirPath.split(QChar('/'));
        QString recPath;
        for(int i= 0; i<list.length(); i++)
        {
            recPath += list[i] + "/";
            QDir recDir(recPath);
            if(!recDir.exists())
                recDir.mkdir(recPath);
        }
    }

}
*/

/*
QString getName(QString reference)
{
    int count = 0;
    QString sName = QString(reference).arg(count);
    while(QFile::exists(sName))
    {
        count++;
        sName = QString(reference).arg(count);
    }

    return sName;
}
*/
/*
QString getFileName(QString sPath, QString sName, QString sExtension)
{
    mkDirRecursive(QDir::currentPath()+"/"+sPath);
    int i= 0;
    QString sFileName(QDir::currentPath()+"/"+sPath+"/"+sName+QString("_%1.").arg(i)+sExtension);;
    while(QFile::exists(sFileName))
    {
        i++;
        sFileName = QDir::currentPath()+"/"+sPath+"/"+sName+QString("_%1.").arg(i)+sExtension;
    }

    return sFileName;
}
*/

/*
string getCurrentTime()
{
    QDateTime time;
    return time.currentDateTime().toString(Qt::TextDate).toStdString();
}
*/

/*QColor GetColour(double v,double vmin,double vmax)
{
   QColor c(255,255,255) ; // white
   double dv;

   if(vmin < 0 || vmax < 0 || v < 0 || v > vmax || v < vmin)
   {
       printf("\nValores negativos v:%f, vmin:%f, vmax:%f\n", v, vmin, vmax);
       return QColor(0,0,0);
   }

   if (v < vmin)
      v = vmin;
   if (v > vmax)
      v = vmax;
   dv = vmax - vmin;

   if (v < (vmin + 0.25 * dv)) {
      c.setRed(0);
      c.setGreenF(4 * (v - vmin) / dv);
   } else if (v < (vmin + 0.5 * dv)) {
      c.setRed(0);
      c.setBlueF(1 + 4 * (vmin + 0.25 * dv - v) / dv);
   } else if (v < (vmin + 0.75 * dv)) {
      c.setRedF( 4 * (v - vmin - 0.5 * dv) / dv );
      c.setBlue(0);
   } else {
      c.setGreenF(1 + 4 * (vmin + 0.75 * dv - v) / dv);
      c.setBlue(0);
   }

   return c;
}
*/

//void ReadFloatImage (QVector<QVector<float> >& image, QString sFileName)
//{
	/*
    int numdims, width, height, depth;
    FILE *fin = fopen(sFileName.toStdString().c_str(),"r");
    fscanf(fin, "FLOAT\n");
    fscanf(fin, "%d\n%d %d %d\n", &numdims, &width, &height, &depth );

    float values[width*height];
    fread(values, sizeof(float), width*height, fin );

    image.resize(width);
    for(int row = 0; row < width; row++)
    {
        image[row].resize(height);
        for(int col = 0; col < height; col++)
        {
            image[row][height-col-1] = values[row*height+col];
        }
    }
	*/

    /*
    QFile in(sFileName);

    if(!in.open(QFile::ReadOnly))
        return;

    QTextStream outFloats(&in);

    QString sDummy;
    qint64 pos = in.pos();
    outFloats >> sDummy >> sDummy;

    int width, height;

     pos = in.pos();
 //   outFloats >> width >> height >> sDummy;

    image.resize(width);

    pos = in.pos();
    QDataStream inFloatsBin(&in);

    for(int row = 0; row < 100; row++)
    {
        QChar value;
        inFloatsBin >> value;
        int jjj = 0;
    }

    for(int row = 0; row < width; row++)
    {
        image[row].resize(height);
        for(int col = 0; col < height; col++)
        {
            float value;
            inFloatsBin >> value;
             pos = in.pos();
            if(value < 3000 && value > 0)
                int pararse = 0;
            image[row][height-col-1] = value;
        }
    }
    in.close();
*/
//}

/*void WriteFloatImage (QVector<QVector<float> >& image, QString sFileName)
{
    QFile out(sFileName);

    if(!out.open(QFile::WriteOnly))
        return;


    int imageWidth, imageHeight;
    imageWidth = image.size();

    if(imageWidth > 0)
        imageHeight = image[0].size();
    else
        imageHeight = 0;

    QTextStream outFloats(&out);
    outFloats << QString("FLOAT") << endl << "3" << endl<< imageWidth << " " << imageHeight << " 1"<< endl;
    outFloats.flush();

    QDataStream outFloatsBin(&out);

    for(int row = 0; row < imageWidth; row++)
    {
        for(int col = 0; col < imageHeight; col++)
        {
            outFloatsBin << image[row][imageHeight - col - 1];
        }
    }
    out.close();
}
*/

/*
void computeFaceTBNBasis(QVector<QVector3D>& faceVerts, QVector<QVector2D>& faceUV, int idx, QVector3D& tangent, QVector3D& binormal, QVector3D& normal) {

    int idxPt0 = idx, idxPt1 = idx+1 , idxPt2 = idx+2;

    QVector3D p02  = faceVerts[idxPt2]- faceVerts[idxPt0];           //p2-p1
    QVector3D p01  = faceVerts[idxPt1]- faceVerts[idxPt0];           //p3-p1

    QVector2D uv01 = faceUV[idxPt1]- faceUV[idxPt0];       //uv2-uv1
    QVector2D uv02 = faceUV[idxPt2]- faceUV[idxPt0];       //uv3-uv1

    tangent = uv02.x()*p02 + uv01.x()*p01;
    binormal = uv02.y()*p02 + uv01.y()*p01;

    tangent.normalize();
    binormal.normalize();

    normal = QVector3D::crossProduct(tangent,binormal);
    normal.normalize();
}
*/
