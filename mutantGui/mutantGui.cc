#include "mutantGui.h"
static const double Pi = 3.14159265358979323846264338327950288419717;
static double TwoPi = 2.0 * Pi;

using namespace qtGui;

/***
 * Implementation of mutantGui method
 ******************************************************************************/

mutantGui::mutantGui(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::mutantGui),
  widthBox(2), depthBox(4), nodesBox(1), eDensityBox(4), eLengthBox(2),
  nbClusterBox(2), maxIterBox(1),
  aes(true), backprop(true), fft(true), bfs(true), gemm(true), kmp(true),
  md(true), nw(true), sort(true), spmv(true), stenc(true), viterbi(true),
  randMap(true), showProp(true), appsGraph()
{
  ui->setupUi(this);
  this->setFocusPolicy(Qt::StrongFocus);
  display = new drawDisplay(ui->display);
}

mutantGui::~mutantGui()
{
  delete ui;
}

std::vector<uint> mutantGui::computeBenchList(){
  std::vector<uint> bench;
  if(aes){bench.push_back(ID_AES);}
  if(viterbi){bench.push_back(ID_VITERBI);}
  if(bfs){bench.push_back(ID_BFSBULK);}
  if(backprop){bench.push_back(ID_BACKPROP);}
  if(fft){bench.push_back(ID_FFTSTRIDED);}
  if(gemm){bench.push_back(ID_GEMMBLOCKED);}
  if(kmp){bench.push_back(ID_KMP);}
  if(md){bench.push_back(ID_MDKNN);}
  if(nw){bench.push_back(ID_NW);}
  if(sort){bench.push_back(ID_SORTMERGE);}
  if(stenc){bench.push_back(ID_STENC2D);}
  if(spmv){bench.push_back(ID_SPMV);}

  return(bench);
}

void mutantGui::keyPressEvent(QKeyEvent *event){
  print_DMGUI("key pressed" << event->text().toStdString() << "\n");
  //    graph->keyPressEvent(event);
}

void drawDisplay::refresh(uint depth, uint width, nodes ** mtxNodes, uint *edges, bool showProp){
  int step = (true==showProp)?(2*B_SCALE):(B_SCALE);

  //clear scene and redefine dimensions
  if(NULL == scene){delete scene;}
  scene = new QGraphicsScene(this);
  scene->setItemIndexMethod(QGraphicsScene::NoIndex);
  int xLen = step*(2+ width);
  int yLen = step*(2+ depth);
  scene->setSceneRect(-xLen, -yLen, xLen, yLen);
  setScene(scene);

  gNode *nodes[depth*width];

  for(uint w=0; w<width; w++){
    for(uint d=0; d<depth; d++){
      if(NULL != mtxNodes[_2d_node(w, d)]){
        nodes[_2d_node(w, d)] = new gNode(this, showProp, mtxNodes[_2d_node(w, d)]);
        scene->addItem(nodes[_2d_node(w, d )]);
        int x = step*((int)w - width) - (step/2);
        int y = step*((int)d - depth) - (step/2);
        nodes[_2d_node(w, d)]->setPos(x, y);
        print_DMGUI("Draw node ["<< w << ", " << d<< "]\n"
                    << "\t @"<< x << ", " << y<< "\n");
      }
    }
  }

  for(uint w=0; w<width; w++){ // loop on node
    for(uint d=0; d<depth; d++){
      for(uint tw=0; tw<width; tw++){ //loop on potential trgt nodes
        for(uint td=0; td<depth; td++){
          if( 0< edges[_outEdges(w, d, tw, td)]){
            print_DMGUI("edge between [" <<d << ", "<< w << "] and ["
                        << td << ", "<< tw << "] \n");
            scene->addItem(new gEdge(nodes[_2d_node(w, d)],
                                   nodes[_2d_node(tw, td)]));
          }
        }
      }
    }
  }

}

/***
 * Implementation of drawDisplay method
 ******************************************************************************/
drawDisplay::drawDisplay(QWidget * parent)
  : QGraphicsView(parent), scene(NULL)
{
  setCacheMode(CacheBackground);
  setViewportUpdateMode(BoundingRectViewportUpdate);
  setRenderHint(QPainter::Antialiasing);
  setTransformationAnchor(AnchorUnderMouse);
  scale(qreal(0.8), qreal(0.8));
  setSizeAdjustPolicy(SizeAdjustPolicy::AdjustToContents);
  setMinimumSize(1670, 1080);
  setWindowTitle(tr("Mutant application graph"));

}

void drawDisplay::drawBackground(QPainter *painter, const QRectF &rect)
{
  Q_UNUSED(rect);

  // Shadow
  QRectF sceneRect = this->sceneRect();
  QRectF rightShadow(sceneRect.right(), sceneRect.top() + 5, 5, sceneRect.height());
  QRectF bottomShadow(sceneRect.left() + 5, sceneRect.bottom(), sceneRect.width(), 5);
  if (rightShadow.intersects(rect) || rightShadow.contains(rect))
    painter->fillRect(rightShadow, Qt::darkGray);
  if (bottomShadow.intersects(rect) || bottomShadow.contains(rect))
    painter->fillRect(bottomShadow, Qt::darkGray);

  // Fill
  QLinearGradient gradient(sceneRect.topLeft(), sceneRect.bottomRight());
  gradient.setColorAt(0, Qt::white);
  gradient.setColorAt(1, Qt::lightGray);
  painter->fillRect(rect.intersected(sceneRect), gradient);
  painter->setBrush(Qt::NoBrush);
  painter->drawRect(sceneRect);

  // Text
  QRectF textRect(sceneRect.left() + 4, sceneRect.top() + 4,
                  sceneRect.width() - 4, sceneRect.height() - 4);
  QString message(tr(""));

  QFont font = painter->font();
  font.setBold(true);
  font.setPointSize(14);
  painter->setFont(font);
  painter->setPen(Qt::lightGray);
  painter->drawText(textRect.translated(2, 2), message);
  painter->setPen(Qt::black);
  painter->drawText(textRect, message);
}

void drawDisplay::keyPressEvent(QKeyEvent *event)
{
  switch (event->key()) {
  case Qt::Key_Up:
    break;
  case Qt::Key_Down:
    break;
  case Qt::Key_Left:
    break;
  case Qt::Key_Right:
    break;
  case Qt::Key_Plus:
    zoomIn();
    break;
  case Qt::Key_Minus:
    zoomOut();
    break;
  case Qt::Key_Space:
  case Qt::Key_Enter:
    break;
  default:
    QGraphicsView::keyPressEvent(event);
  }
}

#ifndef QT_NO_WHEELEVENT
void drawDisplay::wheelEvent(QWheelEvent *event)
{
  scaleView(pow((double)2, -event->delta() / 240.0));
}
#endif
void drawDisplay::zoomIn()
{
  scaleView(qreal(1.2));
}
void drawDisplay::zoomOut()
{
  scaleView(1 / qreal(1.2));
}

void drawDisplay::scaleView(qreal scaleFactor)
{
  qreal factor = transform().scale(scaleFactor, scaleFactor).mapRect(QRectF(0, 0, 1, 1)).width();
  if (factor < 0.07 || factor > 100)
    return;

  scale(scaleFactor, scaleFactor);
}

/***
 * Implementation of gNode method
 ******************************************************************************/
gNode::gNode(drawDisplay *display, bool _show, nodes* _prop)
  : graph(display), properties(_prop), showProp(_show), popup(NULL)
{
  setFlag(ItemIsMovable);
  setFlag(ItemSendsGeometryChanges);
  setCacheMode(DeviceCoordinateCache);
  setZValue(-1);
}

void gNode::addEdge(gEdge *edge)
{
  edgeList << edge;
  edge->adjust();
}

QList<gEdge *> gNode::edges() const
{
  return edgeList;
}

QRectF gNode::boundingRect() const
{
  qreal adjust = 12;
  return QRectF( -(NODE_X) -adjust, -(NODE_Y) -adjust, 3*NODE_X + adjust, 3*NODE_Y + adjust);
}

QPainterPath gNode::shape() const
{
  QPainterPath path;
  path.addEllipse( -(NODE_X/2), -(NODE_Y/2), NODE_X, NODE_Y);
  return path;
}

void gNode::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *)
{
  //draw shadow
  uint shadow = 4;
  painter->setPen(Qt::NoPen);
  painter->setBrush(Qt::darkGray);
  painter->drawEllipse( -(NODE_X/3)+shadow, -(NODE_Y/3)+shadow, NODE_X/2, NODE_Y/2);


  //Set front color and gradient
    QRadialGradient gradient(-5, -5, 30);
  gradient.setColorAt(1, QColor(Qt::blue).light(120));
  gradient.setColorAt(0, QColor(Qt::yellow).light(120));
  painter->setBrush(gradient);

  //draw front task shape
  painter->setPen(QPen(Qt::black, 0));
  painter->drawEllipse( -(NODE_X/3), -(NODE_Y/3), NODE_X/2, NODE_Y/2);
  //ShortName
  std::string buffer = "T" + std::to_string(properties->id);
  painter->setPen(QPen(Qt::blue, 0));
  painter->drawText(-(NODE_X/3), -(NODE_Y/3), NODE_X/2, NODE_Y/2, Qt::AlignHCenter|Qt::AlignVCenter, buffer.c_str());

  //draw properties box
  if(showProp){
    painter->setPen(Qt::black);
    painter->setBrush(Qt::white);
    painter->drawRect((NODE_X/6), -(NODE_Y), 2*NODE_X*(0.9), NODE_Y*(0.8));
    //Task name iteration and target Cl
    std::string name= task_helper.at(properties->task_id) + (properties->hwTask?" [HW]":"");
    painter->drawText((NODE_X/6), -(NODE_Y), 2*NODE_X*(0.9), NODE_Y*(0.8), Qt::AlignHCenter|Qt::AlignTop, name.c_str());
    painter->drawText((NODE_X/6), -(NODE_Y), 2*NODE_X*(0.9), NODE_Y*(0.8), Qt::AlignHCenter|Qt::AlignVCenter,
                      std::to_string(properties->nbIteration).c_str());
    std::string target= (0xff == properties->clTarget)?"unMapped":std::to_string(properties->clTarget);
    painter->drawText((NODE_X/6), -(NODE_Y), 2*NODE_X*(0.9), NODE_Y*(0.8), Qt::AlignHCenter|Qt::AlignBottom,
                      target.c_str());
  }
}

void gNode::updateProp(std::string task, uint clId, uint iter, bool hwState)
{
  //update properties in graph
  uint task_id;
  for (auto it = task_helper.begin(); it != task_helper.end(); ++it ){
    if (it->second == task){
      task_id = it->first;
      break;
    }
  }
  properties->task_id = task_id;
  properties->hwTask = hwState;
  properties->clTarget = clId;
  properties->nbIteration = iter;
  update();
}

void gNode::popupEnded()
{
  popup->close();
  delete popup;
  popup = NULL;
}

QVariant gNode::itemChange(GraphicsItemChange change, const QVariant &value)
{
  switch (change) {
  case ItemPositionHasChanged:
    foreach (gEdge *edge, edgeList)
      edge->adjust();
    // graph->itemMoved();
    break;
  default:
    break;
  };

  return QGraphicsItem::itemChange(change, value);
}

void gNode::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
  update();
  QGraphicsItem::mousePressEvent(event);
}

void gNode::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
  update();
  if(NULL == popup){
    popup = new PopUp(task_helper.at(properties->task_id), properties->clTarget,
                      properties->nbIteration, properties->hwTask);
    QObject::connect(popup, SIGNAL(updateGraph(std::string, uint, uint, bool)),
                     this, SLOT(updateProp(std::string, uint, uint, bool)));
    QObject::connect(popup, SIGNAL(closePopUp()),
                     this, SLOT(popupEnded()));
  }

  popup->show();
  QGraphicsItem::mouseReleaseEvent(event);
}

/***
 * Implementation of gEdge method
 ******************************************************************************/
gEdge::gEdge(gNode *sourceNode, gNode *destNode)
  : arrowSize(10)
{
  setAcceptedMouseButtons(0);
  source = sourceNode;
  dest = destNode;
  source->addEdge(this);
  dest->addEdge(this);
  adjust();
}


void gEdge::adjust()
{
  if (!source || !dest)
    return;

  QLineF line(mapFromItem(source, -(NODE_Y/12), -(NODE_Y/12)), mapFromItem(dest, -(NODE_Y/12), -(NODE_Y/12)));
  qreal length = line.length();

  prepareGeometryChange();

  if (length > qreal(20.)) {
    QPointF edgeOffset((line.dx() * (NODE_Y/4)) / length, (line.dy() * (NODE_Y/4)) / length);
    sourcePoint = line.p2() - edgeOffset;
    destPoint = line.p1() + edgeOffset;
  } else {
    sourcePoint = destPoint = line.p1();
  }
}

QRectF gEdge::boundingRect() const
{
  if (!source || !dest)
    return QRectF();

  qreal penWidth = 1;
  qreal extra = (penWidth + arrowSize) / 2.0;

  return QRectF(sourcePoint, QSizeF(destPoint.x() - sourcePoint.x(),
                                    destPoint.y() - sourcePoint.y()))
    .normalized()
    .adjusted(-extra, -extra, extra, extra);
}

void gEdge::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
  if (!source || !dest)
    return;

  QLineF line(sourcePoint, destPoint);
  if (qFuzzyCompare(line.length(), qreal(0.)))
    return;

  // Draw the line itself
  painter->setPen(QPen(Qt::black, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  painter->drawLine(line);

  // Draw the arrows
  double angle = ::acos(line.dx() / line.length());
  if (line.dy() >= 0)
    angle = TwoPi - angle;

  QPointF sourceArrowP1 = sourcePoint + QPointF(sin(angle + Pi / 3) * arrowSize,
                                                cos(angle + Pi / 3) * arrowSize);
  QPointF sourceArrowP2 = sourcePoint + QPointF(sin(angle + Pi - Pi / 3) * arrowSize,
                                                cos(angle + Pi - Pi / 3) * arrowSize);
  // QPointF destArrowP1 = destPoint + QPointF(sin(angle - Pi / 3) * arrowSize,
                                            // cos(angle - Pi / 3) * arrowSize);
  // QPointF destArrowP2 = destPoint + QPointF(sin(angle - Pi + Pi / 3) * arrowSize,
                                            // cos(angle - Pi + Pi / 3) * arrowSize);

  painter->setBrush(Qt::black);
  painter->drawPolygon(QPolygonF() << line.p1() << sourceArrowP1 << sourceArrowP2);
  // painter->drawPolygon(QPolygonF() << line.p2() << destArrowP1 << destArrowP2);
  painter->drawPolygon(QPolygonF() << line.p1());
}


/***
 * GUI Main application
 ******************************************************************************/
int main(int argc, char **argv)
{
  QApplication app(argc, argv);
  mutantGui *gui = new mutantGui;

  QMainWindow mainWindow;
  mainWindow.setCentralWidget(gui);
  mainWindow.setFocusPolicy(Qt::StrongFocus);

  mainWindow.show();
  return app.exec();
}
