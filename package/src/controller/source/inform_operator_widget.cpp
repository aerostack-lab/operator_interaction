/*!*******************************************************************************************
 *  \file       inform_operator_widget.cpp
 *  \brief      InformOperatorWidget implementation file.
 *  \details    This file implements the InformOperatorWidget class.
 *  \authors    Abraham Carrera.
 *  \copyright  Copyright (c) 2019 Universidad Politecnica de Madrid
 *              All rights reserved
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include <inform_operator_widget.h>
#include "ui_inform_operator_widget.h"

InformOperatorWidget::InformOperatorWidget(QWidget *parent) : QWidget(parent), ui(new Ui::InformOperatorWidget)
{
  ui->setupUi(this);
  layout = ui->gridLayout;
  button_layout = ui->button_layout;
  text_edit = ui->text_edit;
}

InformOperatorWidget::InformOperatorWidget(std::string message, BehaviorInformOperator *behavior)
    : QWidget(0), ui(new Ui::InformOperatorWidget)
{
  ui->setupUi(this);
  layout = ui->gridLayout;
  button_layout = ui->button_layout;
  text_edit = ui->text_edit;
  confirm = new QPushButton("Accept");
  connect(confirm, SIGNAL(clicked()), this, SLOT(confirmation()));
  confirm->setEnabled(true);
  layout->addWidget(confirm, 3, 0, 1, 2, Qt::AlignBottom);
  this->behavior = behavior;
  ui->label->setText(QString::fromUtf8("Message:"));
  text_edit->setPlainText(QString::fromUtf8((message).c_str()));
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer,sizeof(buffer),"%H:%M:%S",timeinfo);
  std::string str(buffer);
  ui->time->setText(QString::fromUtf8("Time: ")+QString::fromUtf8(str.c_str()));

}

InformOperatorWidget::~InformOperatorWidget()
{
  delete mapper;
  delete ui;
}

void InformOperatorWidget::confirmation()
{
  this->QWidget::close();
  behavior->started = false;

}
