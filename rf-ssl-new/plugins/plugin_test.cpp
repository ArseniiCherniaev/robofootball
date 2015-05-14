#include "plugin_test.h"
#include <stdio.h>
#include <exception>
#include <qmath.h>
#include <sstream>
#include <ctime>

testPlugin::testPlugin( FrameBuffer * _buffer, LUT3D * lut, const CameraParameters& camera_params, const RoboCupField& field ,  CMPattern::TeamSelector * _global_team_selector_blue2, CMPattern::TeamSelector * _global_team_selector_yellow2,PluginDetectBallsSettings * settings)
    : VisionPlugin ( _buffer ), camera_parameters ( camera_params ), field ( field )
{
    printf("TEST!\r\n");
    C1 = 50;
    C2 = 20;
    h = 2.2;
    _settings=settings;
    //_have_local_settings=false;
    if ( _settings==0 ) {
      _settings = new PluginDetectBallsSettings();
      //_have_local_settings=true;
    }
    //printf("testPlugin inti\n");
    cl=new imageClust();
    cl->readCovs();


    imProc = new imageProcessing(C1, C2, h);
    m_c = new QList<pixelloc>();
    avg = Mat::zeros(480,640,CV_32FC3);
    count=0;
    isFirstRun = true;
    setColor = -1;
    left = 0;
    top = 0;
    right = 640;
    bottom = 480;
    isFullImageAlgorithm = true;
    drawHeightExamples = false;
}


testPlugin::~testPlugin()
{
   delete cl;
    delete imProc;
}

void testPlugin::mousePressEvent(QMouseEvent *event, pixelloc loc)
{
    //info->setText(QString::number(loc.x)+QString(" ")+QString::number(loc.y));

    //m_c->push_back(loc);
    rgb * source_pointer = (rgb*)(source->getData());
    int r = (int)source_pointer[loc.y * source -> getWidth() + loc.x].r;
    int g = (int)source_pointer[loc.y * source -> getWidth() + loc.x].g;
    int b = (int)source_pointer[loc.y * source -> getWidth() + loc.x].b;

    switch(setColor)
    {
        case 0: colorsForObjects[0].add(r,g,b);
                cout << "Add (" << r << ", " << g << ", " << b << ") as blue" << endl;
                break;
        case 1: colorsForObjects[1].add(r,g,b);
                cout << "Add (" << r << ", " << g << ", " << b << ") as yellow" << endl;
                break;
        case 2: colorsForObjects[2].add(r,g,b);
                cout << "Add (" << r << ", " << g << ", " << b << ") as red" << endl;
                break;
        case 3: colorsForObjects[3].add(r,g,b);
                cout << "Add (" << r << ", " << g << ", " << b << ") as green" << endl;
                break;
        case 4: colorsForObjects[4].add(r,g,b);
                cout << "Add (" << r << ", " << g << ", " << b << ") as orange" << endl;
                break;
        case 5: if (numberOfCountedPoints == 0)
                {
                    if (teamID == 0)
                    {
                        int i = 0;
                        for (; i < (int)blueTeam.size() && robotID != blueTeam.at(i).robotID; i++);
                        if (i < (int)blueTeam.size())
                            cout << "Start editing robot Blue " << robotID << " data" << endl;
                        else
                            cout << "Start matching robot Blue " << robotID << endl;

                    }
                    else
                    {
                        int i = 0;
                        for (; i < (int)yellowTeam.size() && robotID != yellowTeam.at(i).robotID; i++);
                        if (i < (int)yellowTeam.size())
                            cout << "Start editing robot Yellow " << robotID << " data" << endl;
                        else
                            cout << "Start matching robot Yellow " << robotID << endl;
                    }
                    robotX = loc.x;
                    robotY = loc.y;
                    centerColor = FColor();
                    centerColor.add(r,g,b);
                    numberOfCountedPoints++;
                    cout << "Set center of robot as (" << loc.x << ", " << loc.y
                         << "), color of region 0 as (" << r << ", " << g << ", " << b << ")" << endl;
                }
                else
                {
                    cout << "Set color of region "<< numberOfCountedPoints << " as ("
                    << r << ", " << g << ", " << b << ")" << endl;
                    if (numberOfCountedPoints == 1)
                    {
                        redColor = FColor();
                        redColor.add(r,g,b);
                        numberOfCountedPoints++;
                    }
                    else
                    {
                        greenColor = FColor();
                        greenColor.add(r,g,b);
                        setColor = -1;
                        if (teamID == 0)
                        {
                            int i = 0;
                            for (; i < (int)blueTeam.size() && robotID != blueTeam.at(i).robotID; i++);
                            if (i < (int)blueTeam.size())
                                blueTeam.at(i) = RobotFeatures(robotX, robotY, 0.0, centerColor, redColor, greenColor, teamID, robotID);
                            else
                                blueTeam.push_back(RobotFeatures(robotX, robotY, 0.0, centerColor, redColor, greenColor,teamID, robotID));
                        }
                        else
                        {
                            int i = 0;
                            for (; i < (int)yellowTeam.size() && robotID != yellowTeam.at(i).robotID; i++);
                            if (i < (int)yellowTeam.size())
                                yellowTeam.at(i) = RobotFeatures(robotX, robotY, 0.0, centerColor, redColor, greenColor,teamID, robotID);
                            else
                                yellowTeam.push_back(RobotFeatures(robotX, robotY, 0.0, centerColor, redColor, greenColor,teamID, robotID));
                        }
                        cout << "Finished specifying" << endl;
                        numberOfCountedPoints = 0;
                    }
                }
                break;
        case 6: left = loc.x;
                top = loc.y;
                cout << "Set Left-Top Corner as (" << loc.x << ", " << loc.y << ")" << endl;
                break;
        case 7: right = loc.x;
                bottom = loc.y;
                cout << "Set Right-Bottom Corner as (" << loc.x << ", " << loc.y << ")" << endl;
                break;
        default:
                cout << "Color of ("<< loc.x << ", " << loc.y << ") is ("
                     << (int)source_pointer[loc.y * source -> getWidth() + loc.x].r << ", "
                     << (int)source_pointer[loc.y * source -> getWidth() + loc.x].g << ", "
                     << (int)source_pointer[loc.y * source -> getWidth() + loc.x].b << ")" << endl;
    }
}

ProcessResult testPlugin::process(FrameData * data, RenderOptions * options)
{
    source = &(data->video);
    rgb *        source_pointer = (rgb*)(source->getData());

    if (isFirstRun)
    {
        if (drawHeightExamples)
        {
            Mat img = Mat::zeros(source -> getHeight(),source -> getWidth(),CV_8UC3);
            uchar* mat = img.data;
            int width = source -> getWidth();
            for (int i = 0; i < source -> getHeight(); i++)
                for (int j = 0; j < source -> getWidth(); j++)
                {
                    mat[3 * (i * width + j) + 2] = source_pointer[i * width + j].r;
                    mat[3 * (i * width + j) + 1] = source_pointer[i * width + j].g;
                    mat[3 * (i * width + j) + 0] = source_pointer[i * width + j].b;
                }
            circle(img, Point(20, 40), C2/h, Scalar(255,255,255));
            circle(img, Point(20, 40), 2, Scalar(255,255,255));
            circle(img, Point(100, 40), 4.0/5 * C1/h, Scalar(255,255,255));
            circle(img, Point(100, 40), 2, Scalar(255,255,255));
            circle(img, Point(180, 40), C1/h, Scalar(255,255,255));
            circle(img, Point(180, 40), 2, Scalar(255,255,255));
            for (int i = 0; i < source -> getHeight(); i++)
                for (int j = 0; j < source -> getWidth(); j++)
                {
                    source_pointer[i * width + j].r = mat[3 * (i * width + j) + 2];
                    source_pointer[i * width + j].g = mat[3 * (i * width + j) + 1];
                    source_pointer[i * width + j].b = mat[3 * (i * width + j) + 0];
                }
        }
        else
            imProc -> calibrate(isFullImageAlgorithm, source, colorsForObjects);
    }
    else
    {
        clock_t start = std::clock();

        if (isFullImageAlgorithm)
            imProc -> getStartData(source, colorsForObjects, left, top, right, bottom);
        else
            imProc -> getNewData(&blueTeam, &yellowTeam, colorsForObjects[4], source);

        double duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        cout << duration << endl;
    }
     return ProcessingOk;
}

Mat imageClust::Wolf(Mat img, int w, double k){

    printf("start wolf\n");



    Scalar S = sum(img);
    double Smax=S[0]>S[1] ? S[0]:S[1];
    Smax=S[2]>Smax ? S[2]:Smax;

    uchar *mat=img.data;
    int cols = img.cols;
    int rows = img.rows;

    for (int i=0; i < rows; ++i)
        for (int j = 0; j < cols; ++j)
        {
            mat[3*(i*cols + j)+0] *= 2-S[0]/Smax;
            mat[3*(i*cols + j)+1] *= 2-S[1]/Smax;
            mat[3*(i*cols + j)+2] *= 2-S[2]/Smax;
        }


        cvtColor(img,img,CV_BGR2GRAY);

        double minI, maxI;
        minMaxLoc(img, &minI, &maxI);

        img=(img-minI)*255/(maxI-minI);

        img.convertTo(img,CV_32F);

        Mat m, s;
        Mat kernel = Mat::ones(w,w,CV_32F) / (float)(w*w);

        filter2D(img, m, CV_32F, kernel);

        Mat t = Mat(img.rows,img.cols,CV_32F);
        t = img - m;
        filter2D(t.mul(t), s, CV_32F, kernel);
        sqrt(s,s);


        double minS, maxS;
        minMaxLoc(s, &minS, &maxS);

        t = m+k*(s/maxS-1).mul(m-minI);

        Mat binimg = (img>t & s>15);

        Mat se =(Mat_<unsigned char>(4,4) << 0, 1, 1, 0,
                                             1, 1, 1, 1,
                                             1, 1, 1, 1,
                                             0, 1, 1, 0);
        morphologyEx(binimg, binimg, MORPH_OPEN, se);
        return binimg;
}


void testPlugin::calibClick(){

    lock();

    cl->calibrate();
    unlock();

    info->setText("Lal!");
    //layout->update();

}

void testPlugin::startProcess(){
    lock();
    isFirstRun = false;
    unlock();
}

void testPlugin::startSpecifyingRobot(){
    lock();
    setColor = 5;
    teamID = (teamSelector->currentText() == "Blue")? 0 : 1;
    robotID = robotId -> value();
    numberOfCountedPoints = 0;
    unlock();
}

void testPlugin::setColorModeOnBlue(){
    lock();
    if (setColor != 0)
        setColor = 0;
    else
        setColor = -1;
    unlock();
}

void testPlugin::setColorModeOnYellow(){
    lock();
    if (setColor != 1)
        setColor = 1;
    else
        setColor = -1;
    unlock();
}

void testPlugin::setColorModeOnRed(){
    lock();
    if (setColor != 2)
        setColor = 2;
    else
        setColor = -1;
    unlock();
}

void testPlugin::setColorModeOnGreen(){
    lock();
    if (setColor != 3)
        setColor = 3;
    else
        setColor = -1;
    unlock();
}

void testPlugin::setColorModeOnOrange(){
    lock();
    if (setColor != 4)
        setColor = 4;
    else
        setColor = -1;
    unlock();
}

void testPlugin::setLeftTopCorner()
{
    lock();
    setColor = 6;
    unlock();
}

void testPlugin::setRightBottomCorner()
{
    lock();
    setColor = 7;
    unlock();
}

void testPlugin::changeAlgorithmTo()
{
    if (isFullImageAlgorithm)
    {
        isFullImageAlgorithm = false;
        isFirstRun = true;
        changeAlgorithm -> setText("Change to Full Image Algorithm");
    }
    else
    {
        isFullImageAlgorithm = true;
        isFirstRun = true;
        changeAlgorithm -> setText("Change to Local Area Algorithm");
    }
}

void testPlugin::readClick()
{
    cl->readCovs();
    info->setText("Looks like read :)");
}

void imageClust::setImg(Mat &img)
{
    rlist.init();
    reglist.init();
    //Mat tmp1, tmp2;
   // src.convertTo(tmp1,CV_32FC3);
    //img.convertTo(tmp2,CV_32FC3);
    //((Mat)((tmp1+tmp2)/2)).convertTo(src,CV_8UC3);
    //src=(src+img)/2;
    //addWeighted(src,0.5,img,0.5,0.0,src);
    src = img;
//    gray = Wolf(img,15,-0.1);

    gray=Mat(img.size(),CV_8UC1);
    cvtColor(img,gray,CV_BGR2GRAY);
    blur(gray,gray,Size(3,3));
    threshold(gray,gray,thresh,255,CV_THRESH_BINARY);
    //readCovs();

}

void testPlugin::threshChanged(int i)
{
    cl->thresh=i;
    info->setText(QString::number(cl->thresh));
}

void testPlugin::showHeight()
{
    if (openHeightControl -> text()[0] == 'O')
    {
        openHeightControl -> setText("Close height examples");
        drawHeightExamples = true;
    }
    else
    {
        openHeightControl -> setText("Open height examples");
        drawHeightExamples = false;
    }
}

void testPlugin::setHeight()
{
    h = heightGetter -> value();
    imProc -> setH(h);
}

QWidget* testPlugin::getControlWidget()
{
    sb=new QSpinBox();
    sb->setMaximum(255);
    sb->setFixedWidth(40);
    sb->setValue(10);

    teamSelector = new QComboBox;
    teamSelector -> addItem("Blue");
    teamSelector -> addItem("Yellow");
    teamSelector -> setFixedWidth(70);

    robotId = new QSpinBox();
    robotId->setMaximum(11);
    robotId->setMinimum(0);
    robotId->setValue(0);
    robotId->setFixedWidth(40);

    heightGetter = new QDoubleSpinBox();
    heightGetter->setMaximum(5.0);
    heightGetter->setMinimum(0);
    heightGetter->setValue(h);
    heightGetter->setFixedWidth(40);

    openHeightControl = new QToolButton();
    openHeightControl -> setText("Open height examples");

    QWidget * widget=new QWidget();
    layout=new QVBoxLayout();
    QToolButton * calib=new QToolButton();
    QToolButton * read=new QToolButton();

    QToolButton * red=new QToolButton();
    QToolButton * green=new QToolButton();
    QToolButton * yellow=new QToolButton();
    QToolButton * blue=new QToolButton();
    QToolButton * orange=new QToolButton();

    QToolButton * setLeftCorner = new QToolButton();
    QToolButton * setRightCorner = new QToolButton();

    QToolButton * startWork = new QToolButton();
    QToolButton * specifyRobot = new QToolButton();

    changeAlgorithm = new QToolButton();
    changeAlgorithm -> setText("Change to Local Area Algorithm");

    read->setText("Read covs");

    calib->setText("Push me!");

    startWork -> setText("START");
    specifyRobot -> setText("Specify Robot");

    yellow->setText("Yellow");
    blue->setText("Blue");
    red->setText("Red");
    green->setText("Green");
    orange->setText("Orange");

    setLeftCorner -> setText("Set Left-Top Corner");
    setRightCorner -> setText("Set Right-Bottom Corner");

    connect(calib,SIGNAL(clicked(bool)),this,SLOT(calibClick()));
    connect(read,SIGNAL(clicked(bool)),this,SLOT(readClick()));
    connect(sb,SIGNAL(valueChanged(int)),this,SLOT(threshChanged(int)));

    connect(blue,SIGNAL(clicked(bool)),this,SLOT(setColorModeOnBlue()));
    connect(yellow,SIGNAL(clicked(bool)),this,SLOT(setColorModeOnYellow()));
    connect(red,SIGNAL(clicked(bool)),this,SLOT(setColorModeOnRed()));
    connect(green,SIGNAL(clicked(bool)),this,SLOT(setColorModeOnGreen()));
    connect(orange,SIGNAL(clicked(bool)),this,SLOT(setColorModeOnOrange()));

    connect(setLeftCorner,SIGNAL(clicked(bool)),this,SLOT(setLeftTopCorner()));
    connect(setRightCorner,SIGNAL(clicked(bool)),this,SLOT(setRightBottomCorner()));

    connect(startWork,SIGNAL(clicked(bool)),this,SLOT(startProcess()));
    connect(specifyRobot,SIGNAL(clicked(bool)),this,SLOT(startSpecifyingRobot()));

    connect(changeAlgorithm,SIGNAL(clicked(bool)),this,SLOT(changeAlgorithmTo()));

    connect(openHeightControl,SIGNAL(clicked(bool)), this, SLOT(showHeight()));
    connect(heightGetter, SIGNAL(valueChanged(double)), this, SLOT(setHeight()));

    info=new QLabel();
    info->setText(QString::number(sb->value()));

    layout->addWidget(red);
    layout->addWidget(green);
    layout->addWidget(blue);
    layout->addWidget(yellow);
    layout->addWidget(orange);

    layout->addWidget(setLeftCorner);
    layout->addWidget(setRightCorner);

    layout->addWidget(startWork);
    layout -> addWidget(changeAlgorithm);
    layout -> addWidget(teamSelector);
    layout -> addWidget(robotId);
    layout -> addWidget(specifyRobot);

    layout -> addWidget(heightGetter);
    layout -> addWidget(openHeightControl);

    layout->addWidget(info);
    layout->addWidget(calib);
    layout->addWidget(read);

    layout->addWidget(sb);
    widget->setLayout(layout);
    return widget;
}

string testPlugin::getName()
{
    return "Test";
}


imageClust::imageClust()
{
   // src = Mat::zeros(480,640,CV_8UC3);
    //Mat::zeros()
    ball = Ball();               //new
    drawColors[0]=Scalar(255,0,0);
    drawColors[1]=Scalar(0,255,255);
    drawColors[2]=Scalar(0,255,0);
    drawColors[3]=Scalar(255,0,255);
    drawColors[4]=Scalar(0,0,255);

    calib_colors[0][0]=0;
    calib_colors[0][1]=3;
    calib_colors[0][2]=3;
    calib_colors[0][3]=3;
    calib_colors[0][4]=2;

    calib_colors[1][0]=0;
    calib_colors[1][1]=3;
    calib_colors[1][2]=3;
    calib_colors[1][3]=2;
    calib_colors[1][4]=2;

    calib_colors[2][0]=0;
    calib_colors[2][1]=3;
    calib_colors[2][2]=2;
    calib_colors[2][3]=3;
    calib_colors[2][4]=2;

    calib_colors[3][0]=1;
    calib_colors[3][1]=3;
    calib_colors[3][2]=3;
    calib_colors[3][3]=3;
    calib_colors[3][4]=3;
    thresh=10;

        //readCovs();
}

void imageClust::ellipse_calc(double ix, double iy, double ixy)
{
    if ((ixy == 0) && (ix == iy))
    {
        b = sqrt(ix);
        a = sqrt(iy);
        phi = 0;
    }
    double D = sqrt((ix-iy)*(ix-iy) + 4*ixy*ixy);
    b = sqrt((ix+iy - D)/2);
    a = sqrt((ix+iy + D)/2);

    double d = (ix-iy)/D;
    if (d > 0)
        phi = atan(sqrt((1-d)/(1+d)));
    else
        phi = pi/2 - atan(sqrt((1+d)/(1-d)));
    if (ixy < 0)
        phi = -phi;
}

//rle-????????? ??????????????? ???????????
void imageClust::rle()
{
    Run* runs=rlist.runs;
    uchar *mat=gray.data;

    int width=src.cols;
        int rows=src.rows;
    Run r;
    int x,tmp,col;
    int j=0;

        for (int i=0;i<rows;++i)
    {
        r.y=i;
        x=0;
        while(x<width)
        {
                        col=mat[i*width+x];

            tmp=x;
                        while(x<width && mat[i*width+x]==col) ++x;

            r.x=tmp;
            r.width=x-tmp;
            r.parent=j;
            r.color=col;
            runs[j++]=r;
        }
    }
    rlist.used=j;
    printf("used: %d\n",rlist.used);
}

//????????? ??????? ???????? ?? rle-?????????
void imageClust::clust()
{
    //????????? ?? ?????? ?????
    Run * runs=rlist.runs;
    //??? ??????? ????
    Run r1,r2;
    //?????????? ????? ?????
    int used=rlist.used;
    //???????? ??? ?????? ? ?????? ??????, ??????????????
    int l1=0;
    int l2=1;
    //?????? ??????????????? ?????
    int i,j,s;
    //??????? ?????? ?????? ?????? ?????????? ? ??????? ???? ?????? ?????? :)
    while (runs[l2].y==0) ++l2;
    //?????????????? ???? ?????? ? ?????? ?????
    r1=runs[l1];
    r2=runs[l2];
    //?????????? ???-??
    s=l2;
    //???? ??????? ?????? ?????? ?? ?????? ?????? ?????????? ?????
    while (l2<used)
    {
        //???? ? ??? ??? ???? ?????? ????? ? ?? ??????
        if (r1.color==r2.color && r1.color>0)
        {
            //??????? ??????????? ???? ??????
            if ((r1.x-(r2.x+r2.width))*(r2.x-(r1.x+r1.width))>=0)
            {
                //???-?? ????? ??????????
                if (s!=l2)
                {
                    runs[l2].parent = r2.parent = r1.parent;
                    s = l2;
                }
                //???? ???????? ?? ?????????
                else if (r1.parent!=r2.parent)
                {
                    //??????? ???????? ????????? ????? ?????
                    i = r1.parent;
                    while(i != runs[i].parent) i = runs[i].parent;
                    j = r2.parent;
                    while(j != runs[j].parent) j = runs[j].parent;
                    //???????? ???????? ? ?????????? ?????????
                    //(?????, ????? ????????? ???????? ???????
                    //???????????????? ????????????? ?????)
                    //?? ? ????????????? ?????????
                    if (i<j)
                    {
                        runs[j].parent=i;
                        runs[l1].parent = runs[l2].parent = r1.parent = r2.parent = i;
                    }
                    else
                    {
                        runs[i].parent=j;
                        runs[l1].parent = runs[l2].parent = r1.parent = r2.parent = j;
                    }
                }
            }
        }

        //??????? ? ?????????? ????
        i = (r1.x + r1.width) - (r2.x + r2.width);
        if(i >= 0)
        {
            r2 = runs[++l2];
        }
        if(i <= 0)
        {
            r1 = runs[++l1];
        }
    }

    //??????????????? ?????????? ???????????? ????????
    for(i=0; i<used; i++)
    {
        j = runs[i].parent;
        runs[i].parent = runs[j].parent;
    }
    //printf("rle: %d\n",used);
}

//????????? ????????
//?????? - ???????, ?????????? ?? ???????????????
//+?????????? ? ????????? ?????
//+???????????? ?????? ????????
void imageClust::regions()
{
    uchar * mat=src.data;
    int cols=src.cols;

    Run curr;
    Region * regions=reglist.regions;
    Run * runs=rlist.runs;
    int used=rlist.used;
    int n=0;
    int y,parent,width;

    for (int i=0;i<used;++i)
    {
        if (runs[i].color>0)
        {

            curr=runs[i];
            width=curr.width;

            if (curr.parent==i)
            {
               // printf("if %d\n",curr.parent);
                runs[i].region=n;

                regions[n].first_run=i;
                regions[n].area=width;
                regions[n].cx=width*(2*curr.x+width-1)/2;
                regions[n].cy=curr.y*width;
                ++n;
                //printf("n: %d\n",n);
            }

            else
            {
               // printf("else parent:%d\n",curr.parent);
                parent=runs[curr.parent].region;
                runs[i].region=parent;
                //parent=runs[curr.parent].parent;
                //runs[i].parent=parent;
                regions[parent].area+=width;
                regions[parent].cx+=width*(2*curr.x+width-1)/2;
                regions[parent].cy+=curr.y*width;
            }
        }
    }
    reglist.used=n;
    printf("regions1: %d\n",n);

    Region * r=new Region[100];
    int count=0;
    for (int i=0;i<n;++i)
    {
        regions[i].cx/=regions[i].area;
        regions[i].cy/=regions[i].area;
        if (regions[i].area>MIN_AREA && regions[i].area<MAX_AREA)
        {
            r[count]=regions[i];
            ++count;
        }
    }
        delete[] regions;
    reglist.regions=r;
    reglist.used=count;
    regions=reglist.regions;

    printf("regions2: %d\n",n);
    n=reglist.used;
    Region * final_r=new Region[n];

    count =0;
    for (int i=0;i<n;++i)
    {
        curr=runs[regions[i].first_run];
        parent=curr.parent;
        for (int j=0;j<used;++j)
        {
            if (runs[j].parent==parent)
            {
                y=runs[j].y;
                for (int k=runs[j].x;k<runs[j].x+runs[j].width;++k)
                {
                    regions[i].ix+=(k-regions[i].cx)*(k-regions[i].cx);
                    regions[i].iy+=(y-regions[i].cy)*(y-regions[i].cy);
                    regions[i].ixy+=(k-regions[i].cx)*(y-regions[i].cy);

                    regions[i].rgb[0]+=mat[3*(y*cols+k)];
                    regions[i].rgb[1]+=mat[3*(y*cols+k)+1];
                    regions[i].rgb[2]+=mat[3*(y*cols+k)+2];

                }
                //regions[i].color[0]+=mat[runs[j].y*cols+runs[j]

            }
        }
        regions[i].ix/=regions[i].area;
        regions[i].iy/=regions[i].area;
        regions[i].ixy/=regions[i].area;

        regions[i].rgb[0]/=regions[i].area;
        regions[i].rgb[1]/=regions[i].area;
        regions[i].rgb[2]/=regions[i].area;

        ellipse_calc(regions[i].ix,regions[i].iy,regions[i].ixy);
        stringstream ss;
        if (a/b<1.5 && a*b<140)
        {
            final_r[count]=regions[i];
            /*
            ss<<regions[i].first_run;
            putText(src,ss.str(),Point(regions[i].cx,regions[i].cy),CV_FONT_HERSHEY_PLAIN,1,Scalar(0,255,0));
            ss.str("");
            */
            ++count;
        }
    }
        delete[] r;
    reglist.regions=final_r;
    reglist.used=count;
    printf("regions: %d\n",reglist.used);


}

//??????? ??? ??????????? ????????? ????????
void imageClust::recover(Mat &src)
{
     printf("recover0\n");
    uchar * mat=src.data;
    Run curr;
    Run * r=rlist.runs;
    Region * reg=reglist.regions;

    int cols=src.cols;
    int parent;
    int n=reglist.used;
    int run_n=rlist.used;
    int y;


//        for (int i=0;i<n;++i)
//    {
//        curr=r[reg[i].first_run];
//        parent=curr.parent;
//        for (int j=0;j<run_n;++j)
//        {
//            if (r[j].parent==parent)
//            {
//                y=r[j].y;
//                for (int k=r[j].x;k<r[j].x+r[j].width;++k)
//                                {
//                                    mat[3*(y*cols+k)]=255;
//                                     mat[3*(y*cols+k)+1]=255;
//                                      mat[3*(y*cols+k)+2]=255;
//                                    //src.at<Scalar>(k,y)=drawColors[r[j].color];
//                                    //src.at<Scalar>(k,y)=Scalar(255);
//                                }
//            }
//        }
//    }

    printf("recover\n");
    /*for (int i=0;i<n;++i)
    {
        circle(src,Point(reg[i].cx,reg[i].cy),5,drawColors[reg[i].color]);
    }*/
}

//??????????? ?????? ???????? ?? ??????? ???????? ??????????
void imageClust::colors()
{

    Run * runs=rlist.runs;
    Region * regs=reglist.regions;
    Region curr_reg;
    int n=reglist.used;
    int res_col;
    double min=6;
    double curr;
    Mat cov(3,3,CV_64F);
    Mat col(1,3,CV_64F);
    Mat res;
    stringstream ss;

    for (int j=0;j<n;++j)
    {
        curr_reg=regs[j];
        min=6;
        for (int i=0;i<5;++i)
        {
            ss<<i<<".txt";
            ifstream in(ss.str().c_str());
            ss.str("");
            col.at<double>(0,0)=cols[i].at<double>(0,0)-curr_reg.rgb[0];
            col.at<double>(0,1)=cols[i].at<double>(0,1)-curr_reg.rgb[1];
            col.at<double>(0,2)=cols[i].at<double>(0,2)-curr_reg.rgb[2];
            res=col*covs[i]*col.t();
            curr=res.at<double>(0,0);
            if (curr<min)
            {
                min=curr;
                res_col=i;
            }
        }
        if (min<6)
        {
            regs[j].color=res_col;
        }
        else
            regs[j].color=5;
      //  printf("color: %d\n",regs[j].color);
    }
}

//??????????????? ?????????? ?????? ?????????? ?? ??????.
//Nuff said.
void imageClust::readCovs()
{
    stringstream ss;
    for (int i=0;i<5;++i)
    {
        covs[i]=Mat(3,3,CV_64F);
        cols[i]=Mat(1,3,CV_64F);
        ss<<i<<".txt";
        ifstream in(ss.str().c_str());
        ss.str("");

        in>>cols[i].at<double>(0,0)>>cols[i].at<double>(0,1)>>cols[i].at<double>(0,2);
               // printf("%d center: %f %f %f\n",i,cols[i].at<double>(0,0),cols[i].at<double>(0,1),cols[i].at<double>(0,2));
               // printf("%d cov:\n",i);

        for (int k=0;k<3;++k)
        {
              //  printf("\t");
            for (int j=0;j<3;++j)
            {
                in>>covs[i].at<double>(k,j);
                // printf("%f ",covs[i].at<double>(k,j));
            }
           // printf("\n");
        }
    }
}

bool regionCompare (Region i,Region j) { return (i.angle<j.angle); }

void imageClust::invar(Hat &h)
{
   // printf("inver starting\n");
    Region regs[5];

    for (int i=0;i<5;++i)
    {
        regs[i]=h.regs[i];
    }

    double sumx=0;
    double sumy=0;
    for (int i=0;i<5;++i)
    {
        sumx+=regs[i].cx;
        sumy+=regs[i].cy;
    }
    sumx/=5;
    sumy/=5;

    double dist=100;
    double curr_dist;
    int cnt_no=0;
    Point cnt(regs[0].cx,regs[0].cy);

    for (int i=0;i<5;++i)
    {
        curr_dist=sqrt((sumx-regs[i].cx)*(sumx-regs[i].cx)+(sumy-regs[i].cy)*(sumy-regs[i].cy));

        if (curr_dist<dist)
        {

            dist=curr_dist;
            cnt_no=i;
            cnt=Point(regs[i].cx,regs[i].cy);
        }
    }

    //double angle;

    h.regs[0]=regs[cnt_no];

    for (int i=0;i<4;++i)
    {
        if (i>=cnt_no)
            regs[i]=regs[i+1];

        regs[i].cx-=h.regs[0].cx;
        regs[i].cy-=h.regs[0].cy;
        regs[i].angle=atan2(regs[i].cy,regs[i].cx);
    }
    sort(regs,regs+4,regionCompare);
    for (int i=0;i<4;++i)
    {
        regs[i].cx+=h.regs[0].cx;
        regs[i].cy+=h.regs[0].cy;
    }

    int fr1=0;
    int fr2=0;
    int j1,j2;
    dist=0;
    for (int i=0;i<4;++i)
    {
        j1 = i+1>3 ? 0 : i+1;
        j2 = i-1<0 ? 3 : i-1;
        curr_dist=sqrt((double)(regs[i].cx-regs[j1].cx)*(regs[i].cx-regs[j1].cx)+(regs[i].cy-regs[j1].cy)*(regs[i].cy-regs[j1].cy));
        if (curr_dist>dist)
        {
            dist=curr_dist;
            fr1=i;
            fr2=j1;
        }
        curr_dist=sqrt((double)(regs[i].cx-regs[j2].cx)*(regs[i].cx-regs[j2].cx)+(regs[i].cy-regs[j2].cy)*(regs[i].cy-regs[j2].cy));
        if (curr_dist>dist)
        {
            dist=curr_dist;
            fr1=i;
            fr2=j2;
        }
    }

    int tmp=0;
    double a,b,c1;

    a=regs[fr2].cy-regs[fr1].cy;
    b=regs[fr1].cx-regs[fr2].cx;
    c1=regs[fr1].cy*(regs[fr2].cx-regs[fr1].cx)-regs[fr1].cx*(regs[fr2].cy-regs[fr1].cy);

    double val=a*cnt.x+b*cnt.y+c1;
    if (val>0)
    {
        tmp=fr1;
        fr1=fr2;
        fr2=tmp;
    }

    //h.regs[1]=regs[fr1];
    //h.regs[2]=regs[fr2];
    //printf("%d fr2: %d %d\n",h.regs[1].first_run,fr1,fr2);
    if (fr2>fr1 && abs(fr2-fr1)==1 || (fr2<fr1) && abs(fr2-fr1)>1)
    {
        for (int i=1;i<5;++i)
        {

            if (i-fr1<1)
                tmp=4+i-fr1;
            else
                tmp=i-fr1;
            h.regs[tmp]=regs[i-1];
        }
    }

    else
    {
        for (int i=1;i<5;++i)
        {
            if (i-fr1-1<=0)
                tmp=1+abs(i-fr1-1);
            else
                tmp=4-i+fr1;
            h.regs[tmp]=regs[i-1];
        }
    }
    //printf("inver worked\n");
}

void Hat::computeId()
{

    id=(regs[1].color-2)+2*(regs[2].color-2)+4*(regs[3].color-2)+8*(regs[4].color-2);
    id=id>11? 11:id;

}

void imageClust::findHats()
{
    //lock();
    rle();
    //printf("rle worked!\n");
    clust();
   // printf("clust worked!\n");
    regions();
   // printf("regions worked!\n");
    colors();
   // printf("colors worked!\n");
    for (int i=0;i<12;++i)
    {
        blues[i].id=-1;
        yellows[i].id=-1;

    }



    int n=reglist.used;
    Region * regs=reglist.regions;
    //Run * runs=rlist.runs;
    double dist;
    int count=0;
    //int blues_count=0;
   // int yellows_count=0;
    double cx=0;
    double cy=0;
    int thr=21;
    for (int i=0;i<n;++i)
    {

        if (regs[i].color<2)
        {
           // printf("got it!\n");
            Hat h;
            count=0;
            h.regs[count++]=regs[i];
            //circle(src,Point(h.regs[0].cx,h.regs[0].cy),1,Scalar(255,255,255),2);
            //circle(src,Point(h.regs[0].cx,h.regs[0].cy),thr,drawColors[regs[i].color],1);
            cx=0;
            cy=0;
            for (int j=0;j<n;++j)
            {
                dist=sqrt((regs[i].cx-regs[j].cx)*(regs[i].cx-regs[j].cx)+(regs[i].cy-regs[j].cy)*(regs[i].cy-regs[j].cy));
               // printf("%d-%d color: %d dist: %f\n",i,j,regs[j].color,dist);

                if (dist<thr && (regs[j].color==2 || regs[j].color==3) && i!=j && count<5)
                {
                   // printf("%d %d %d\n",count++,i,j);
                    h.regs[count++]=regs[j];
                    cx+=regs[j].cx;
                    cy+=regs[j].cy;
                  //  printf("added %d\n",j);
                }
            }

            cx/=4;
            cy/=4;
            //circle(src,Point(cx,cy),1,drawColors[regs[i].color],1);

           // printf("after dist\n");

            if (count==5)
            {

                invar(h);
                h.x=h.regs[0].cx;
                h.y=h.regs[0].cy;
                h.color=h.regs[0].color;

                double dy=(h.regs[3].cy-h.regs[4].cy - h.regs[1].cy+h.regs[2].cy)/2;
                double dx=-(h.regs[3].cx-h.regs[4].cx - h.regs[1].cx+h.regs[2].cx)/2;
                h.phi=atan2(dy,dx);
               // printf("phi: %f\n",h.phi);
                h.computeId();
                printf("%d color id: %d\n",h.color,h.id);
                if (h.color==0)
                    blues[h.id]=h;
                else
                    yellows[h.id]=h;
                //hats[hats_count++]=h;

                //line(src,Point(h.regs[1].cx,h.regs[1].cy),Point(h.regs[2].cx,h.regs[2].cy),Scalar(0,regs[i].color*255,255),2);
                //circle(src,Point(h.regs[2].cx,h.regs[2].cy),5,Scalar(0,0,255),2);
                //line(src,Point(h.regs[3].cx,h.regs[3].cy),Point(h.regs[4].cx,h.regs[4].cy),Scalar(0,regs[i].color*255,255),2);
            }


       }
        if (regs[i].color==4)
        {
            ball.x=regs[i].cx;
            ball.y=regs[i].cy;
            ball.rgb[0] = regs[i].rgb[0];
            ball.rgb[1] = regs[i].rgb[1];
            ball.rgb[2] = regs[i].rgb[2];
            cout << "Color of Ball: " << regs[i].rgb[0] << " " << regs[i].rgb[1] << " " << regs[i].rgb[2] << endl;
        }
    }
    //imwrite("img.png",src);
    printf("findhats worked!\n");
     printf("yo!\n");

  // unlock();
}


void imageClust::calibrate()
{
    //imwrite("out.png",src);
//        rle();
//        printf("calib rle worked!\n");
//        clust();
//        printf("calib clust worked!\n");
//        regions();
//        printf("calib regions worked!\n");

    //printf("-*-*-*-*-*-*calib worked lol!-*-*-*-*-*-*\n");
       // system("pause");


        // --------------------------------- //
        int src_cols=src.cols;
        uchar * mat=src.data;

        Region * regs=reglist.regions;
        Run * runs=rlist.runs;

        int n=reglist.used;
        printf("calib regs: %d\n",n);
        double cx=0;
        double cy=0;
        double dist=200;
        double curr_dist;
        int ball;
        stringstream ss;

        for (int i=0;i<n;++i)
        {
                cx+=regs[i].cx;
                cy+=regs[i].cy;
        }

        cx/=n;
        cy/=n;
        printf("%f %f\n",cx,cy);
        //circle(src,Point(cx,cy),3,Scalar(0,255,0),-1);

        for (int i=0;i<n;++i)
        {
                curr_dist=sqrt((regs[i].cx-cx)*(regs[i].cx-cx)+(regs[i].cy-cy)*(regs[i].cy-cy));
                if (curr_dist<dist)
                {
                        dist=curr_dist;
                        ball=i;
                }
        }

        cx=regs[ball].cx;
        cy=regs[ball].cy;
        printf("%f %f\n",cx,cy);
//        //circle(src,Point(regs[ball].cx,regs[ball].cy),5,Scalar(0,0,255),-1);
        Hat hats[4];
        for (int i=0;i<n;++i)
        {
                if (i==ball) continue;
                if (regs[i].cx-cx<0)
                        if (regs[i].cy-cy<0)
                                hats[0].add(regs[i]);
                        else
                                hats[3].add(regs[i]);
                else
                        if (regs[i].cy-cy<0)
                                hats[1].add(regs[i]);
                        else
                        {
                            printf("2 %d\n",i);
                                hats[2].add(regs[i]);
                        }
        }

        int parent;
        for (int i=0;i<4;++i)
        {printf("h %d %d\n",i,hats[i].used);

                invar(hats[i]);
                for (int j=0;j<5;++j)
                {
//                    printf("i: %d j: %d\n",i,j);
                        parent=runs[hats[i].regs[j].first_run].parent;
                        for (int k=0;k<rlist.used;++k)
                        {
//                            printf("%d %d\n",rlist.used,k);
                                if (runs[k].parent==parent)
                                {
//                                    printf("%d %d %d \n",rlist.used,i,j);
                                        for (int s=runs[k].x;s<runs[k].x+runs[k].width;++s)
                                        {

                                                if (checkColor(calib_colors[i][j],mat+3*(runs[k].y*src_cols+s)))
                                                        colorset[calib_colors[i][j]].add(mat+3*(runs[k].y*src_cols+s));
                                        }
                                }
                        }
                }
        }
      //  system("pause");
        printf("after colorcheck\n");

        parent=runs[regs[ball].first_run].parent;
        for (int k=0;k<rlist.used;++k)
        {
            if (runs[k].parent==parent)
            {
                for (int s=runs[k].x;s<runs[k].x+runs[k].width;++s)
                {
                    if (checkColor(4,mat+3*(runs[k].y*src_cols+s)))
                        colorset[4].add(mat+3*(runs[k].y*src_cols+s));
                }
            }
        }

        int k=0;
        for (int i=0;i<colorset[k].used;++i)
            printf("%d blue color: %d %d %d\n",i,colorset[k].bgr[i][0],colorset[k].bgr[i][1],colorset[k].bgr[i][2]);



        for (int j=0;j<5;++j)
        {
                ss<<j<<".txt";
                ofstream out(ss.str().c_str());

                cols[j]=Mat(1,3,CV_64F);
                covs[j]=Mat(3,3,CV_64F);
                double bgr[3]={0,0,0};
                Mat cov,mu;
                Mat colors(colorset[j].used,3,CV_8U);
                uchar * color_ptr=colors.data;
                int color_size=colorset[j].used;
                for (int i=0;i<color_size;++i)
                {
                    color_ptr[3*i]=colorset[j].bgr[i][0];
                    color_ptr[3*i+1]=colorset[j].bgr[i][1];
                    color_ptr[3*i+2]=colorset[j].bgr[i][2];

                    bgr[0]+=colorset[j].bgr[i][0];
                    bgr[1]+=colorset[j].bgr[i][1];
                    bgr[2]+=colorset[j].bgr[i][2];
                }
                cols[j].at<double>(0,0)=bgr[0]/=color_size;
                cols[j].at<double>(0,1)=bgr[1]/=color_size;
                cols[j].at<double>(0,2)=bgr[2]/=color_size;

                out<<cols[j].at<double>(0,0)<<" "<<cols[j].at<double>(0,1)<<" "<<cols[j].at<double>(0,2)<<"\n";

                calcCovarMatrix(colors,cov,mu,CV_COVAR_NORMAL|CV_COVAR_ROWS|CV_COVAR_SCALE);
                cov.at<double>(0,0)+=0.01;
                cov.at<double>(1,1)+=0.01;
                cov.at<double>(2,2)+=0.01;

                covs[j]=cov.inv();
                for (int i=0;i<3;++i)
                {
                    for (int k=0;k<3;++k)
                        out<<covs[j].at<double>(i,k)<<" ";
                    out<<"\n";
                }

                printf("%d covar: %f %f %f\n",j,covs[j].at<double>(0,0),covs[j].at<double>(0,1),covs[j].at<double>(0,2));
                printf("%d covar: %f %f %f\n",j,covs[j].at<double>(1,0),covs[j].at<double>(1,1),covs[j].at<double>(1,2));
                printf("%d covar: %f %f %f\n",j,covs[j].at<double>(2,0),covs[j].at<double>(2,1),covs[j].at<double>(2,2));
                printf("--------\n");

                ss.str("");

        }



}

bool imageClust::checkColor(int num,uchar * color)
{

 //   bool hack=0;
 //   printf("check %d %d\n",num,colorset[num].used);
    for (int i=0;i<colorset[num].used;++i)
    {
//        printf("%d checkcolro!\n",i);
        if (colorset[num].bgr[i][0]==color[0] && colorset[num].bgr[i][1]==color[1] && colorset[num].bgr[i][2]==color[2])
        {
            return false;
        }
    }

        return true;
}


imageProcessing::imageProcessing(double C1, double C2, double h)
{ 
    radiusOfVotingForCentralCircle = C2/h;
    radiusOfVotingForSideCircles = C1/h;
    radiusOfRobotSector = radiusOfVotingForSideCircles + 5;
    numberOfThreads = 2;
    this -> C1 = C1;
    this -> C2 = C2;
    this -> h = h;
    frameNumber = 0;
}

void imageProcessing::setH(double h)
{
    this -> h = h;
    radiusOfVotingForCentralCircle = C2/h;
    radiusOfVotingForSideCircles = C1/h;
    radiusOfRobotSector = radiusOfVotingForSideCircles + 5;
}

imageProcessing::~imageProcessing()
{
    delete source_pointer;
}


//Find coordinates and angle for both teams of robots (already existed)
void imageProcessing::getNewData(RobotFeatures& robot)
{
    int oldX = (int)(robot.x);
    int oldY = (int)(robot.y);

    int left = (oldX - radiusOfRobotSector >= 0)? oldX - radiusOfRobotSector : 0;
    int right = (oldX + radiusOfRobotSector < source->getWidth())? oldX + radiusOfRobotSector : source->getWidth() - 1;
    int top = (oldY - radiusOfRobotSector >= 0)? oldY - radiusOfRobotSector : 0;
    int bottom = (oldY + radiusOfRobotSector < source->getHeight())? oldY + radiusOfRobotSector : source->getHeight() - 1;

    int height = bottom - top + 1;
    int width = right - left + 1;

    Mat img=Mat::zeros(height, width,CV_8UC3);
    uchar* mat = img.data;

    #pragma omp parallel for
    for (int i=top;i<=bottom;++i)
        for (int j=left;j<=right;++j)
        {
            mat[3*((i - top)*width+(j - left))+2]=source_pointer[i*source->getWidth()+j].r;
            mat[3*((i - top)*width+(j - left))+1]=source_pointer[i*source->getWidth()+j].g;
            mat[3*((i - top)*width+(j - left))+0]=source_pointer[i*source->getWidth()+j].b;
        }


    double newColorsRecorder[3][4];
    #pragma omp parallel for
    for (int i = 0; i < 3; i++)
    {
        newColorsRecorder[i][0] = 0;
        newColorsRecorder[i][1] = 0;
        newColorsRecorder[i][2] = 0;
        newColorsRecorder[i][3] = 0;
    }

    int* resultOfVoting = new int[height * width];

    #pragma omp parallel for
    for (int i = 0; i < height; i++)
        for (int j = 0; j < width; j++)
            resultOfVoting[i * width + j] = 0;
    int maximum = 0;

    for (int i = 1; i < height - 1; i++)
        for (int j = 1; j < width - 1; j++)
        {
            bool alreadyVoted = false;
            int r = (int)mat[3 * (i * width + j) + 2];
            int g = (int)mat[3 * (i * width + j) + 1];
            int b = (int)mat[3 * (i * width + j) + 0];

            if (!alreadyVoted && robot.centerColor.contains(r,g,b))
            {
                newColorsRecorder[0][0] = (newColorsRecorder[0][0] * newColorsRecorder[0][3] + b)/(newColorsRecorder[0][3] + 1);
                newColorsRecorder[0][1] = (newColorsRecorder[0][1] * newColorsRecorder[0][3] + g)/(newColorsRecorder[0][3] + 1);
                newColorsRecorder[0][2] = (newColorsRecorder[0][2] * newColorsRecorder[0][3] + r)/(newColorsRecorder[0][3] + 1);
                newColorsRecorder[0][3]++;

                alreadyVoted = true;

                for (int x = max(0, i - radiusOfVotingForCentralCircle); x <= min(i + radiusOfVotingForCentralCircle, height - 1); x++)
                {
                    double r = sqrt((double)radiusOfVotingForCentralCircle * radiusOfVotingForCentralCircle - (x - i) * (x - i));
                    for (int y = max(0.0, j - r); y <= min(j + r, (double)width - 1); y++)
                    {
                        resultOfVoting[x * width + y] += 2;
                        if (resultOfVoting[x * width + y] > maximum)
                            maximum = resultOfVoting[x * width + y];
                    }
                }
            }

            if (!alreadyVoted && robot.red.contains(r,g,b))
            {
                newColorsRecorder[1][0] = (newColorsRecorder[0][0] * newColorsRecorder[0][3] + b)/(newColorsRecorder[0][3] + 1);
                newColorsRecorder[1][1] = (newColorsRecorder[0][1] * newColorsRecorder[0][3] + g)/(newColorsRecorder[0][3] + 1);
                newColorsRecorder[1][2] = (newColorsRecorder[0][2] * newColorsRecorder[0][3] + r)/(newColorsRecorder[0][3] + 1);
                newColorsRecorder[1][3]++;

                alreadyVoted = true;

                for (int x = max(0, i - radiusOfVotingForCentralCircle); x <= min(i + radiusOfVotingForCentralCircle, height - 1); x++)
                {
                    double r = sqrt((double)radiusOfVotingForCentralCircle * radiusOfVotingForCentralCircle - (x - i) * (x - i));
                    for (int y = max(0.0, j - r); y <= min(j + r, (double)width - 1); y++)
                    {
                        resultOfVoting[x * width + y] += 1;
                        if (resultOfVoting[x * width + y] > maximum)
                            maximum = resultOfVoting[x * width + y];
                    }
                }
            }

            if (!alreadyVoted && robot.green.contains(r,g,b))
            {
                newColorsRecorder[2][0] = (newColorsRecorder[0][0] * newColorsRecorder[0][3] + b)/(newColorsRecorder[0][3] + 1);
                newColorsRecorder[2][1] = (newColorsRecorder[0][1] * newColorsRecorder[0][3] + g)/(newColorsRecorder[0][3] + 1);
                newColorsRecorder[2][2] = (newColorsRecorder[0][2] * newColorsRecorder[0][3] + r)/(newColorsRecorder[0][3] + 1);
                newColorsRecorder[2][3]++;

                alreadyVoted = true;

                for (int x = max(0, i - radiusOfVotingForCentralCircle); x <= min(i + radiusOfVotingForCentralCircle, height - 1); x++)
                {
                    double r = sqrt((double)radiusOfVotingForCentralCircle * radiusOfVotingForCentralCircle - (x - i) * (x - i));
                    for (int y = max(0.0, j - r); y <= min(j + r, (double)width - 1); y++)
                    {
                        resultOfVoting[x * width + y] += 1;
                        if (resultOfVoting[x * width + y] > maximum)
                            maximum = resultOfVoting[x * width + y];
                    }
                }
            }
        }

    double centerX = 0;
    double centerY = 0;
    int numberOfCorrectlyVotedPoints = 0;
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            if (resultOfVoting[i * width + j] >= 0.8 * maximum)
            {
                centerX = (centerX * numberOfCorrectlyVotedPoints + i) / (numberOfCorrectlyVotedPoints + 1);
                centerY = (centerY * numberOfCorrectlyVotedPoints + j) / (numberOfCorrectlyVotedPoints + 1);
                numberOfCorrectlyVotedPoints++;
            }
        }
    }

    bool angles[360];
    #pragma omp parallel for
    for (int i = 0; i < 360; i++)
        angles[i] = false;

    for (int i = 1; i < height - 1; i++)
        for (int j = 1; j < width - 1; j++)
        {
            if ((i - centerX) * (i - centerX) + (j - centerY) * (j - centerY) <= 16.0/25 * radiusOfVotingForSideCircles * radiusOfVotingForSideCircles)
            {
                int r = (int)mat[3 * (i * width + j) + 2];
                int g = (int)mat[3 * (i * width + j) + 1];
                int b = (int)mat[3 * (i * width + j) + 0];
                bool alreadyVoted = false;
                if (!alreadyVoted && (robot.red.contains(r,g,b) || robot.green.contains(r,g,b)))
                {
                    double x = centerX - i;
                    double y = j - centerY;
                    alreadyVoted = true;
                    double angle = std::atan2(y, x);
                    angle *= 180/pi;
                    if (angle < 0)
                        angle = 360 + angle;
                    angles[(int)(angle + 0.5) % 360] = true;
                }
            }
        }

    int maximumLength = 0;
    int leftBoard = 0;
    int rightBoard = 0;

    bool stop = false;
    for (int i = 0; (i < 360) && (!stop); i++)
        if (angles[i])
        {
            int j = (i + 1) % 360;
            if (j == 0)
                stop = true;
            for (; (j != i) && !angles[j]; j = (j + 1) % 360)
                if (j == 359)
                    stop = true;
            int length = (j > i)? j - i : 360 + j - i;
            if (length > maximumLength)
            {
                maximumLength = length;
                leftBoard = i;
                rightBoard = j;
            }
            i  = j - 1;
       }

    //ofstream myfile;
    //myfile.open ("example.txt");
    //for (int i = 0; i < 360; i++)
    //    myfile << i << " - " << angles[i] << endl;
    //myfile.close();

    double direction = (rightBoard > leftBoard)? (rightBoard + leftBoard)/2.0 : (360 + rightBoard + leftBoard)/2.0;

    line(imageBuffer, Point(left + centerY + 30.0 * std::sin(direction * pi / 180), top + centerX - 30.0 * std::cos(direction * pi / 180)), Point(left + centerY, top + centerX), Scalar(255, 255, 255));
    circle(imageBuffer, Point(left + centerY, top + centerX), 3, Scalar(255, 255, 255));
    circle(imageBuffer, Point(left + centerY, top + centerX), 12, Scalar(255, 255, 255));
    //circle(imageBuffer, Point(left + centerY, top + centerX), 4.0/5 * radiusOfVotingForSideCircles, Scalar(255, 255, 255));

    //ЗДЕСЬ МОЖНО ДОБАВИТЬ КОД ДЛЯ ПЕРЕСЧЕТА ЦВЕТОВ

    robot.x = left + centerY;
    robot.y = top + centerX;
    robot.phi = direction * pi / 180;
    delete [] resultOfVoting;
}


//Find coordinates and angle for both teams of robots (already existed) and ball
void imageProcessing::getNewData(std::deque<RobotFeatures> *blueTeam, std::deque<RobotFeatures> *yellowTeam, FColor orange, ImageInterface *source)
{
    omp_set_num_threads(numberOfThreads);
    this -> source = source;
    imageBuffer = Mat::zeros(source -> getHeight(),source -> getWidth(),CV_8UC3);
    uchar* mat = imageBuffer.data;

    source_pointer = (rgb*)(source->getData());


    #pragma omp parallel for
    for (int i=0;i<480;++i)
        for (int j=0;j<640;++j)
        {
            mat[3*(i*640+j)+2]=source_pointer[i*source->getWidth()+j].r;
            mat[3*(i*640+j)+1]=source_pointer[i*source->getWidth()+j].g;
            mat[3*(i*640+j)+0]=source_pointer[i*source->getWidth()+j].b;
        }

    blur(imageBuffer, imageBuffer, Size(3, 3));

    for (int i = 0; i < (int)(blueTeam -> size()); i++)
       getNewData(blueTeam -> at(i));

    for (int i = 0; i < (int)(yellowTeam -> size()); i++)
        getNewData(yellowTeam -> at(i));

    double ballX = 0.0;
    double ballY = 0.0;
    int num = 0;
    #pragma omp parallel for
    for (int i=0;i<480;++i)
        for (int j=0;j<640;++j)
        {
            if (orange.contains((int)mat[3 * (i * 640 + j) + 2], (int)mat[3 * (i * 640 + j) + 1], (int)mat[3 * (i * 640 + j) + 0]))
            {
                ballX = (ballX * num + i)/(num + 1);
                ballY = (ballY * num + j)/(num + 1);
                num++;
            }
        }
    circle(imageBuffer, Point(ballY, ballX), 3, Scalar(255, 255, 255));
    circle(imageBuffer, Point(ballY, ballX), 12, Scalar(255, 255, 255));

    #pragma omp parallel for
    for (int i=0;i<480;++i)
        for (int j=0;j<640;++j)
        {
            source_pointer[i*source->getWidth()+j].r= mat[3*(i*640+j)+2];
            source_pointer[i*source->getWidth()+j].g= mat[3*(i*640+j)+1];
            source_pointer[i*source->getWidth()+j].b= mat[3*(i*640+j)+0];
        }
    //stringstream ss;
    //ss << frameNumber;
    //string str = ss.str();
    //string name = string("record/") + str + string(".png");

    //cvSaveImage(name.c_str(), &(IplImage(imageBuffer)));
    //frameNumber++;
}

//Help find colors for FullImage Algorithm(!)
void imageProcessing::calibrate(bool isFirstAlgorithm, ImageInterface* source, FColor* colors)
{
    if (isFirstAlgorithm)
    {
        this -> source = source;
        source_pointer = (rgb*)(source->getData());
        int width = source -> getWidth();
        int height = source -> getHeight();
        Mat img = Mat::zeros(height, width,CV_8UC3);
        uchar* mat = img.data;
        for (int i = 0; i < height; i++)
            for (int j = 0; j < width; j++)
            {
                mat[3 *(i * width + j) + 2] = source_pointer[i*width+j].r;
                mat[3 *(i * width + j) + 1] = source_pointer[i*width+j].g;
                mat[3 *(i * width + j) + 0] = source_pointer[i*width+j].b;
            }
        blur(img,img,Size(3,3));
        for (int i=0;i<height;++i)
            for (int j=0;j<width;++j)
            {
                int r = (int)mat[3 *(i * width + j) + 2];
                int g = (int)mat[3 *(i * width + j) + 1];
                int b = (int)mat[3 *(i * width + j) + 0];
                //blue
                if (colors[0].contains(r,g,b))
                {
                    mat[3 *(i * width + j) + 2] = 0;
                    mat[3 *(i * width + j) + 1] = 0;
                    mat[3 *(i * width + j) + 0] = 255;
                }

                //yellow
                if (colors[1].contains(r,g,b))
                {
                    mat[3 *(i * width + j) + 2] = 255;
                    mat[3 *(i * width + j) + 1] = 255;
                    mat[3 *(i * width + j) + 0] = 0;
                }

                //red
                if (colors[2].contains(r,g,b))
                {
                    mat[3 *(i * width + j) + 2] = 255;
                    mat[3 *(i * width + j) + 1] = 0;
                    mat[3 *(i * width + j) + 0] = 0;
                }

                //green
                if (colors[3].contains(r,g,b))
                {
                    mat[3 *(i * width + j) + 2] = 0;
                    mat[3 *(i * width + j) + 1] = 255;
                    mat[3 *(i * width + j) + 0] = 0;
                }

                //orange
                if (colors[4].contains(r,g,b))
                {
                    mat[3 *(i * width + j) + 2] = 255;
                    mat[3 *(i * width + j) + 1] = 128;
                    mat[3 *(i * width + j) + 0] = 0;
                }
            }
        for (int i = 0; i < height; i++)
            for (int j = 0; j < width; j++)
            {
                source_pointer[i*width+j].r = mat[3 *(i * width + j) + 2];
                source_pointer[i*width+j].g = mat[3 *(i * width + j) + 1];
                source_pointer[i*width+j].b = mat[3 *(i * width + j) + 0];
            }
    }
}


//Find centers of robots and ball for full image 640x480
void imageProcessing::getStartData(ImageInterface* source, FColor* colorsOfObjects, int left, int top, int right, int bottom)
{
    omp_set_num_threads(numberOfThreads);
    this -> source = source;
    source_pointer = (rgb*)(source->getData());
    int width = source -> getWidth();
    int height = source -> getHeight();
    imageBuffer = Mat::zeros(height, width,CV_8UC3);
    uchar* mat = imageBuffer.data;

    #pragma omp parallel for
    for (int i=0;i<height;++i)
        for (int j=0;j<width;++j)
        {
            mat[3*(i*width+j)+2]=source_pointer[i*width+j].r;
            mat[3*(i*width+j)+1]=source_pointer[i*width+j].g;
            mat[3*(i*width+j)+0]=source_pointer[i*width+j].b;
        }
    int* colors = new int[height * width];
    blur(imageBuffer,imageBuffer,Size(3,3));

    #pragma omp parallel for
    for (int i=top;i<bottom;++i)
        for (int j=left;j<right;++j)
        {
            int r = mat[3*(i*width+j)+2];
            int g = mat[3*(i*width+j)+1];
            int b = mat[3*(i*width+j)+0];

            colors[i * width + j] = -1;
            for (int k = 0; k < 5; k++)
                if (colorsOfObjects[k].contains(r,g,b))
                {
                    colors[i * width + j] = k;
                    break;
                }
        }

    int* resultOfVoting = new int[height * width];
    int* resultOfVotingForBall = new int[height * width];

    #pragma omp parallel for
    for (int i = 0; i < height; i++)
        for (int j = 0; j < width; j++)
        {
            resultOfVoting[i * width + j] = 0;
            resultOfVotingForBall[i * width + j] = 0;
        }
    int maximum = 0;
    int maximumForBall = 0;

    for (int numberOfStrip = 0; numberOfStrip < 4; numberOfStrip++)
        #pragma omp parallel for
        for (int strip = numberOfStrip; strip < (bottom - top)/radiusOfVotingForSideCircles; strip += 4)
            for (int i = top + radiusOfVotingForSideCircles * strip; i < top + radiusOfVotingForSideCircles * (strip + 1); i++)
                for (int j=left+1;j<right-1;++j)
                {
                    if (colors[i * width + j] > -1)
                    {
                            if (colors[i * width + j] == 2 || colors[i * width + j] == 3)
                            {
                                for (int x = max(0, i - radiusOfVotingForSideCircles); x <= min(i + radiusOfVotingForSideCircles, height - 1); x++)
                                {
                                    double r = sqrt((double)radiusOfVotingForSideCircles * radiusOfVotingForSideCircles - (x - i) * (x - i));
                                    for (int y = max(0.0, j - r); y <= min(j + r, (double)width - 1); y++)
                                    {
                                        resultOfVoting[x * width + y] += 1;
                                        if (resultOfVoting[x * width + y] > maximum)
                                            maximum = resultOfVoting[x * width + y];
                                    }
                                }
                            }
                            if (colors[i * width + j] == 0 || colors[i * width + j] == 1)
                            {
                                for (int x = max(0, i - radiusOfVotingForCentralCircle); x <= min(i + radiusOfVotingForCentralCircle, height - 1); x++)
                                {
                                    double r = sqrt((double)radiusOfVotingForCentralCircle * radiusOfVotingForCentralCircle - (x - i) * (x - i));
                                    for (int y = max(0.0, j - r); y <= min(j + r, (double)width - 1); y++)
                                    {
                                        resultOfVoting[x * width + y] += 3;
                                        if (resultOfVoting[x * width + y] > maximum)
                                            maximum = resultOfVoting[x * width + y];
                                    }
                                }
                            }

                            if (colors[i * width + j] == 4)
                            {
                                for (int x = max(0, i - radiusOfVotingForCentralCircle); x <= min(i + radiusOfVotingForCentralCircle, height - 1); x++)
                                {
                                    double r = sqrt((double)radiusOfVotingForCentralCircle * radiusOfVotingForCentralCircle - (x - i) * (x - i));
                                    for (int y = max(0.0, j - r); y <= min(j + r, (double)width - 1); y++)
                                    {
                                        resultOfVotingForBall[x * width + y] += 1;
                                        if (resultOfVotingForBall[x * width + y] > maximumForBall)
                                            maximumForBall = resultOfVotingForBall[x * width + y];
                                    }
                                }
                            }
                    }
                }

    deque<Scalar> centersOfRobots;
    deque<Scalar> centersOfBalls;

    for (int i=0;i<height;++i)
        for (int j=0;j<width;++j)
        {
//            mat[3 * (i * width + j) + 2] = (int)(255 * (double)resultOfVoting[i * width + j]/maximum);
//            mat[3 * (i * width + j) + 1] = (int)(255 * (double)resultOfVoting[i * width + j]/maximum);
//            mat[3 * (i * width + j) + 0] = (int)(255 * (double)resultOfVoting[i * width + j]/maximum);
            if (resultOfVoting[i * width + j] > 0.7 * maximum)
            {
                bool stop = false;
                for (int k = 0; k < (int)centersOfRobots.size() && !stop; k++)
                    if ((i - centersOfRobots.at(k)[1]) * (i - centersOfRobots.at(k)[1]) +
                            (j - centersOfRobots.at(k)[0]) * (j - centersOfRobots.at(k)[0]) < 225)
                    {
                        stop = true;
                        centersOfRobots.at(k)[0] = (centersOfRobots.at(k)[0] * centersOfRobots.at(k)[2] + j)/(centersOfRobots.at(k)[2] + 1);
                        centersOfRobots.at(k)[1] = (centersOfRobots.at(k)[1] * centersOfRobots.at(k)[2] + i)/(centersOfRobots.at(k)[2] + 1);
                        centersOfRobots.at(k)[2] += 1.0;
                    }
                if (!stop)
                    centersOfRobots.push_back(Scalar(j, i, 1.0));
            }

            if (resultOfVotingForBall[i * width + j] > 0.7 * maximumForBall)
            {
                bool stop = false;
                for (int k = 0; k < (int)centersOfBalls.size() && !stop; k++)
                {
                    if ((i - centersOfBalls.at(k)[1]) * (i - centersOfBalls.at(k)[1]) +
                            (j - centersOfBalls.at(k)[0]) * (j - centersOfBalls.at(k)[0]) < 225)
                    {
                        stop = true;
                        centersOfBalls.at(k)[0] = (centersOfBalls.at(k)[0] * centersOfBalls.at(k)[2] + j)/(centersOfBalls.at(k)[2] + 1);
                        centersOfBalls.at(k)[1] = (centersOfBalls.at(k)[1] * centersOfBalls.at(k)[2] + i)/(centersOfBalls.at(k)[2] + 1);
                        centersOfBalls.at(k)[2] += 1.0;
                    }
                }
                if (!stop)
                {
                    centersOfBalls.push_back(Scalar(j, i, 1.0));
                }
            }
        }

    for (int i = 0; i < (int)centersOfRobots.size(); i++)
    {
        double x = centersOfRobots.at(i)[0];
        double y = centersOfRobots.at(i)[1];
        circle(imageBuffer, Point(x, y), 3, Scalar(255, 255, 255));
        circle(imageBuffer, Point(x, y), 12, Scalar(255, 255, 255));
    }

    for (int i = 0; i < (int)centersOfBalls.size(); i++)
    {
        double x = centersOfBalls.at(i)[0];
        double y = centersOfBalls.at(i)[1];
        circle(imageBuffer, Point(x, y), 3, Scalar(255, 255, 255));
        circle(imageBuffer, Point(x, y), 12, Scalar(255, 255, 255));
    }

    #pragma omp parallel for
    for (int i = 0; i < height; i++)
        for (int j = 0; j < width; j++)
        {
            source_pointer[i * width + j].r = mat[3 * (i * width + j) + 2];
            source_pointer[i * width + j].g = mat[3 * (i * width + j) + 1];
            source_pointer[i * width + j].b = mat[3 * (i * width + j) + 0];
        }

//    stringstream ss;
//    ss << frameNumber;
//    string str = ss.str();
//    string name = string("record/") + str + string(".png");

//    cvSaveImage(name.c_str(), &(IplImage(imageBuffer)));
//    frameNumber++;

    delete [] colors;
    delete [] resultOfVoting;
    delete [] resultOfVotingForBall;
}
