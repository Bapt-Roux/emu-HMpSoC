#include "popup.h"
#include "ui_popup.h"

PopUp::PopUp(std::string _task, uint _clId, uint _iter, bool _hwState, QWidget *parent) :
  QWidget(parent), ui(new Ui::PopUp),
  clId(_clId), iter(_iter), hwState(_hwState),
  aes(false), backprop(false), fft(false), bfs(false), gemm(false), kmp(false),
  md(false), nw(false), sort(false), spmv(false), stenc(false), viterbi(false)
{
  // Init UI and connect signals
  ui->setupUi(this);
  QObject::connect(this, SIGNAL(setClId(int)),
                   ui->clIdSpinBox, SLOT(setValue(int)));
  QObject::connect(this, SIGNAL(setIter(int)),
                   ui->iterSpinBox, SLOT(setValue(int)));

  //convert name to tick box and setup default state
  if (_task =="Aes"){
    aes = true;
    ui->aes->setCheckState(Qt::Checked);
  }else if (_task =="Viterbi"){
    viterbi = true;
    ui->viterbi->setCheckState(Qt::Checked);
  }else if (_task =="Bfs bulk"){
    bfs = true;
    ui->bfs->setCheckState(Qt::Checked);
  }else if (_task =="Backprop"){
    backprop = true;
    ui->backprop->setCheckState(Qt::Checked);
  }else if (_task =="FFT strided"){
    fft = true;
    ui->fft->setCheckState(Qt::Checked);
  }else if (_task =="Gemm blocked"){
    gemm = true;
    ui->gemm->setCheckState(Qt::Checked);
  }else if (_task =="Kmp"){
    kmp = true;
    ui->kmp->setCheckState(Qt::Checked);
  }else if (_task =="Md"){
    md = true;
    ui->md->setCheckState(Qt::Checked);
  }else if (_task =="Nw"){
    nw = true;
    ui->nw->setCheckState(Qt::Checked);
  }else if (_task =="Sort merge"){
    sort = true;
    ui->sort->setCheckState(Qt::Checked);
  }else if (_task =="Stencil"){
    stenc = true;
    ui->stenc->setCheckState(Qt::Checked);
  }else if (_task =="Spmv"){
    spmv = true;
    ui->spmv->setCheckState(Qt::Checked);
  }
  //setup default spinBox value and HW
  emit setClId(clId);
  emit setIter(iter);
  if (hwState)
    ui->HwCheckBox->setCheckState(Qt::Checked);
}

PopUp::~PopUp()
{
  delete ui;
}

void PopUp::on_CancelButton_clicked()
{ // close popup
  emit closePopUp();
}

void PopUp::on_OkButton_clicked()
{ // applied change and close
  std::string task;
  if(aes){task = "Aes";}
  else if(viterbi){task = "Viterbi";}
  else if(bfs){task = "Bfs bulk";}
  else if(backprop){task = "Backprop";}
  else if(fft){task = "FFT strided";}
  else if(gemm){task = "Gemm blocked";}
  else if(kmp){task = "Kmp";}
  else if(md){task = "Md";}
  else if(nw){task = "Nw";}
  else if(sort){task = "Sort merge";}
  else if(spmv){task = "Spmv";}
  else if(stenc){task = "Stencil";}
  emit updateGraph(task, clId, iter, hwState);
  emit closePopUp();
}
