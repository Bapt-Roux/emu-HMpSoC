#ifndef MUTANT_GUI_H
#define MUTANT_GUI_H

#ifdef MGUI_DEBUG
#   define print_DMGUI(message)                 \
  do {std::cout << message;}                    \
  while (false)
#else
#   define print_DMGUI(message)                 \
  do {} while (false)
#endif

#define GETNAME(name) std::string(#name)

#include <QTranslator>
#include <QMainWindow>
#include <QKeyEvent>
#include <QApplication>
#include <QTime>
#include <QGraphicsView>
#include <QMessageBox>
#include <QFileDialog>

#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QPainter>
#include <QStyleOption>
#include <QGraphicsItem>
#include <QList>
#include <QObject>

#include <math.h>

#include <iostream>
#include <vector>

#include "ui_mutantGui.h"
#include "popup.h"
#include "mutantGen.h"
#include "machSuite_cls_helper.h"

#define B_SCALE 100
#define NODE_X 60
#define NODE_Y 60

namespace qtGui {
 class gEdge;
 class drawDisplay;
  class gNode : public QObject, public QGraphicsItem
  {
    Q_OBJECT
  public:
    gNode(drawDisplay *display, bool _show, nodes *_prop);

    void addEdge(gEdge *edge);
    QList<gEdge *> edges() const;

    QRectF boundingRect() const override;
    QPainterPath shape() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

  public slots:
    void updateProp(std::string task, uint clId, uint iter, bool hwState);
    void popupEnded();

  protected:
    QVariant itemChange(GraphicsItemChange change, const QVariant &value) override;
    void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;

  private:
    nodes *properties;
    QList<gEdge *> edgeList;
    QPointF newPos;
    drawDisplay *graph;
    bool showProp;
    PopUp *popup;
  };

  class gEdge : public QGraphicsItem
  {
  public:
    gEdge(gNode *sourceNode, gNode *destNode);

    gNode *sourceNode() const{ return source;}
    gNode *destNode() const{return dest;}
    void adjust();

  protected:
    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

  private:
    gNode *source, *dest;
    QPointF sourcePoint;
    QPointF destPoint;
    qreal arrowSize;
  };

  class drawDisplay: public QGraphicsView
  {
    Q_OBJECT

  public:
    drawDisplay(QWidget *parent = 0);
    void refresh(uint depth, uint width, nodes ** mtxNodes, uint *edges, bool showProp);
 public slots:
    // void shuffle();
    void zoomIn();
    void zoomOut();
    void keyPressEvent(QKeyEvent *event) override;

 protected:
    void drawBackground(QPainter *painter, const QRectF &rect) override;
    #ifndef QT_NO_WHEELEVENT
        void wheelEvent(QWheelEvent *event) override;
    #endif
        void scaleView(qreal scaleFactor);

  private:
    QGraphicsScene *scene;
  };

  class mutantGui : public QMainWindow
  {
    Q_OBJECT

    Ui::mutantGui* ui;
    //Spin Box value
    int widthBox, depthBox, nodesBox, eDensityBox, eLengthBox;
    int nbClusterBox, maxIterBox;
    //Tick Box value
    bool aes, backprop, fft, bfs, gemm, kmp, md, nw, sort, spmv, stenc, viterbi;
    bool randMap, showProp;

    //Mutant generator and display functions
    mutantGen appsGraph;
    drawDisplay *display = NULL;

    void keyPressEvent(QKeyEvent *event) override;

  public:
    explicit mutantGui(QWidget *parent = 0);
    ~mutantGui();

  private slots:
    //Buttons clicked actions
    void on_generate_clicked(){
      appsGraph.generate(depthBox, widthBox, nodesBox, eDensityBox, eLengthBox, nbClusterBox, maxIterBox, computeBenchList(), randMap);
      display->refresh(appsGraph.getDepth(), appsGraph.getWidth(), appsGraph.getNodes(), appsGraph.getEdges(),
                       showProp);
    };
    void on_reDraw_clicked(){
      display->refresh(appsGraph.getDepth(), appsGraph.getWidth(), appsGraph.getNodes(), appsGraph.getEdges(),
                       showProp);
    };
    void on_save_clicked(){
      QString filePath = QFileDialog::getSaveFileName();
      appsGraph.save(filePath.toStdString());
    };
    void on_load_clicked(){
      QString filePath = QFileDialog::getOpenFileName();
      appsGraph.load(filePath.toStdString());
      display->refresh(appsGraph.getDepth(), appsGraph.getWidth(), appsGraph.getNodes(), appsGraph.getEdges(),
                       showProp);
    };

    void on_codeGen_clicked(){
      // appsGraph.codeGeneration();
    };

    //Spin box actions
    void on_widthBox_valueChanged(int arg1){ widthBox = arg1;};
    void on_depthBox_valueChanged(int arg1){ depthBox = arg1;};
    void on_nodesBox_valueChanged(int arg1){ nodesBox = arg1;};
    void on_eDensityBox_valueChanged(int arg1){eDensityBox = arg1;};
    void on_eLengthBox_valueChanged(int arg1){ eLengthBox = arg1;};
    void on_maxIter_valueChanged(int arg1){ maxIterBox = arg1;};
    void on_nbCluster_valueChanged(int arg1){ nbClusterBox = arg1;};

    //Tick box action
    void on_aes_stateChanged(int arg1){ aes = (2==arg1)?true:false;};
    void on_backprop_stateChanged(int arg1){ backprop = (2==arg1)?true:false;};
    void on_fft_stateChanged(int arg1){ fft = (2==arg1)?true:false;};
    void on_bfs_stateChanged(int arg1){ bfs = (2==arg1)?true:false;};
    void on_gemm_stateChanged(int arg1){ gemm = (2==arg1)?true:false;};
    void on_kmp_stateChanged(int arg1){ kmp = (2==arg1)?true:false;};
    void on_md_stateChanged(int arg1){ md = (2==arg1)?true:false;};
    void on_nw_stateChanged(int arg1){ nw = (2==arg1)?true:false;};
    void on_sort_stateChanged(int arg1){ sort = (2==arg1)?true:false;};
    void on_spmv_stateChanged(int arg1){ spmv = (2==arg1)?true:false;};
    void on_stenc_stateChanged(int arg1){ stenc = (2==arg1)?true:false;};
    void on_viterbi_stateChanged(int arg1){ viterbi = (2==arg1)?true:false;};
    std::vector<uint> computeBenchList();
    void on_rmEnable_stateChanged(int arg1){ randMap = (2==arg1)?true:false;};
    void on_showProp_stateChanged(int arg1){ showProp = (2==arg1)?true:false;};
  };
}
#endif // MUTANT_GUI_H
