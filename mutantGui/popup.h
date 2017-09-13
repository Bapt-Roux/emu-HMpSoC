#ifndef POPUP_H
#define POPUP_H

#include <QWidget>

namespace Ui {
class PopUp;
}

class PopUp : public QWidget
{
    Q_OBJECT

public:
  explicit PopUp(std::string task, uint clId, uint iter, bool hwState,QWidget *parent = 0);
    ~PopUp();

signals:
  void updateGraph(std::string task, uint clId, uint iter, bool hw);
  void closePopUp();
  // gui default val
  void setClId(int val);
  void setIter(int val);

private slots:
  // Buttons actions
  void on_CancelButton_clicked();
  void on_OkButton_clicked();

  //Spin box actions
  void on_clIdSpinBox_valueChanged(int arg1){clId = arg1;};
  void on_iterSpinBox_valueChanged(int arg1){iter = arg1;};

  //Tick box action
  void on_HwCheckBox_stateChanged(int arg1){ hwState = (2==arg1)?true:false;};
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

private:
  Ui::PopUp *ui;
  uint clId, iter;
  bool hwState;
  bool aes, backprop, fft, bfs, gemm, kmp, md, nw, sort, spmv, stenc, viterbi;
};

#endif // POPUP_H
