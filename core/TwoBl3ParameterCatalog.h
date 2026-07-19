#ifndef TWO_BL3_PARAMETER_CATALOG_H
#define TWO_BL3_PARAMETER_CATALOG_H

#include <QString>
#include <QVector>

struct TwoBl3ParameterDescriptor
{
    quint16 id = 0;
    quint8 type = 0;
    QString name;
    QString menuPath;
    QString effect;
    double step = 1.0;
    bool tracking = false;
    bool searchable = true;
};

namespace TwoBl3ParameterCatalog
{
const QVector<TwoBl3ParameterDescriptor>& all();
const TwoBl3ParameterDescriptor* find(quint16 id);
double valueFromBits(quint8 type, quint32 bits);
quint32 bitsFromValue(quint8 type, double value);
QString formatValue(quint8 type, double value);
}

#endif
