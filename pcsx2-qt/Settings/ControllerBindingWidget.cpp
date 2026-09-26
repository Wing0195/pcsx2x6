// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

#include <QtCore/QDir>
#include <QtCore/QEvent>
#include <QtCore/QStringList>
#include <QtGui/QFont>
#include <QtWidgets/QInputDialog>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QScrollArea>
#include <algorithm>
#include "fmt/format.h"

#include "common/Console.h"
#include "common/StringUtil.h"

#include "pcsx2/Host.h"
#include "pcsx2/SIO/Pad/Pad.h"
#include "pcsx2/DEV9/ACJV.h"

#include "Settings/ControllerBindingWidget.h"
#include "Settings/ControllerSettingsWindow.h"
#include "Settings/ControllerSettingWidgetBinder.h"
#include "Settings/SettingsWindow.h"
#include "QtHost.h"
#include "QtUtils.h"
#include "SettingWidgetBinder.h"

#include "ui_USBBindingWidget_Buzz.h"
#include "ui_USBBindingWidget_DenshaCon.h"
#include "ui_USBBindingWidget_DrivingForce.h"
#include "ui_USBBindingWidget_Gametrak.h"
#include "ui_USBBindingWidget_GTForce.h"
#include "ui_USBBindingWidget_GunCon2.h"
#include "ui_USBBindingWidget_RealPlay.h"
#include "ui_USBBindingWidget_RyojouhenCon.h"
#include "ui_USBBindingWidget_ShinkansenCon.h"
#include "ui_USBBindingWidget_TranceVibrator.h"

ControllerBindingWidget::ControllerBindingWidget(QWidget* parent, ControllerSettingsWindow* dialog, u32 port)
	: QWidget(parent)
	, m_dialog(dialog)
	, m_config_section(fmt::format("Pad{}", port + 1))
	, m_port_number(port)
{
	m_ui.setupUi(this);
	m_ui.groupBox->setTitle(tr("Controller Port %1").arg(port + 1));

	populateControllerTypes();
	onTypeChanged();

	ControllerSettingWidgetBinder::BindWidgetToInputProfileString(m_dialog->getProfileSettingsInterface(),
		m_ui.controllerType, m_config_section, "Type", Pad::GetControllerInfo(Pad::GetDefaultPadType(port))->name);

	connect(m_ui.controllerType, &QComboBox::currentIndexChanged, this, &ControllerBindingWidget::onTypeChanged);
	connect(m_ui.bindings, &QPushButton::clicked, this, &ControllerBindingWidget::onBindingsClicked);
	connect(m_ui.settings, &QPushButton::clicked, this, &ControllerBindingWidget::onSettingsClicked);
	connect(m_ui.macros, &QPushButton::clicked, this, &ControllerBindingWidget::onMacrosClicked);
	connect(m_ui.automaticBinding, &QPushButton::clicked, this, &ControllerBindingWidget::onAutomaticBindingClicked);
	connect(m_ui.clearBindings, &QPushButton::clicked, this, &ControllerBindingWidget::onClearBindingsClicked);
}

ControllerBindingWidget::~ControllerBindingWidget() = default;

QIcon ControllerBindingWidget::getIcon() const
{
	return m_bindings_widget->getIcon();
}

void ControllerBindingWidget::populateControllerTypes()
{
	for (const auto& [name, display_name] : Pad::GetControllerTypeNames())
	{
		const std::string_view sv(name);
		if (sv == "None" || sv == "DualShock2")
			m_ui.controllerType->addItem(QString::fromUtf8(display_name), QString::fromUtf8(name));
	}
}

void ControllerBindingWidget::onTypeChanged()
{
	const bool is_initializing = (m_ui.stackedWidget->count() == 0);
	const std::string type_name = m_dialog->getStringValue(
		m_config_section.c_str(), "Type", Pad::GetControllerInfo(Pad::GetDefaultPadType(m_port_number))->name);
	const Pad::ControllerInfo* cinfo = Pad::GetControllerInfoByName(type_name);
	if (!cinfo)
	{
		Console.Error(fmt::format("Invalid controller type name '{}' in config, ignoring.", type_name));
		cinfo = Pad::GetControllerInfo(Pad::ControllerType::NotConnected);
	}
	m_controller_type = cinfo->type;

	if (m_bindings_widget)
	{
		m_ui.stackedWidget->removeWidget(m_bindings_widget);
		delete m_bindings_widget;
		m_bindings_widget = nullptr;
	}
	if (m_settings_widget)
	{
		m_ui.stackedWidget->removeWidget(m_settings_widget);
		delete m_settings_widget;
		m_settings_widget = nullptr;
	}
	if (m_macros_widget)
	{
		m_ui.stackedWidget->removeWidget(m_macros_widget);
		delete m_macros_widget;
		m_macros_widget = nullptr;
	}

	const bool has_settings = (!cinfo->settings.empty());
	const bool has_macros = (!cinfo->bindings.empty());
	m_ui.settings->setEnabled(has_settings);
	m_ui.macros->setEnabled(has_macros);

	if (cinfo->type == Pad::ControllerType::DualShock2)
	{
		m_bindings_widget = ControllerBindingWidget_DualShock2::createInstance(this);
	}
	else if (cinfo->type == Pad::ControllerType::Guitar)
	{
		m_bindings_widget = ControllerBindingWidget_Guitar::createInstance(this);
	}
	else if (cinfo->type == Pad::ControllerType::Jogcon)
	{
		m_bindings_widget = ControllerBindingWidget_Jogcon::createInstance(this);
	}
	else if (cinfo->type == Pad::ControllerType::Negcon)
	{
		m_bindings_widget = ControllerBindingWidget_Negcon::createInstance(this);
	}
	else if (cinfo->type == Pad::ControllerType::Popn)
	{
		m_bindings_widget = ControllerBindingWidget_Popn::createInstance(this);
	}
	else
	{
		m_bindings_widget = new ControllerBindingWidget_Base(this);
	}

	m_ui.stackedWidget->addWidget(m_bindings_widget);
	m_ui.stackedWidget->setCurrentWidget(m_bindings_widget);

	if (has_settings)
	{
		m_settings_widget = new ControllerCustomSettingsWidget(
			cinfo->settings, m_config_section, std::string(), "Pad", getDialog(), m_ui.stackedWidget);
		m_ui.stackedWidget->addWidget(m_settings_widget);
	}

	if (has_macros)
	{
		m_macros_widget = new ControllerMacroWidget(this);
		m_ui.stackedWidget->addWidget(m_macros_widget);
	}

	updateHeaderToolButtons();

	// no need to do this on first init, only changes
	if (!is_initializing)
		m_dialog->updateListDescription(m_port_number, this);
}

void ControllerBindingWidget::updateHeaderToolButtons()
{
	const QWidget* current_widget = m_ui.stackedWidget->currentWidget();
	const QSignalBlocker bindings_sb(m_ui.bindings);
	const QSignalBlocker settings_sb(m_ui.settings);
	const QSignalBlocker macros_sb(m_ui.macros);

	const bool is_bindings = (current_widget == m_bindings_widget);
	m_ui.bindings->setChecked(is_bindings);
	m_ui.automaticBinding->setEnabled(is_bindings);
	m_ui.clearBindings->setEnabled(is_bindings);
	m_ui.macros->setChecked(current_widget == m_macros_widget);
	m_ui.settings->setChecked((current_widget == m_settings_widget));
}

void ControllerBindingWidget::onBindingsClicked()
{
	m_ui.stackedWidget->setCurrentWidget(m_bindings_widget);
	updateHeaderToolButtons();
}

void ControllerBindingWidget::onSettingsClicked()
{
	if (!m_settings_widget)
		return;

	m_ui.stackedWidget->setCurrentWidget(m_settings_widget);
	updateHeaderToolButtons();
}

void ControllerBindingWidget::onMacrosClicked()
{
	if (!m_macros_widget)
		return;

	m_ui.stackedWidget->setCurrentWidget(m_macros_widget);
	updateHeaderToolButtons();
}

void ControllerBindingWidget::onAutomaticBindingClicked()
{
	QMenu menu(this);
	bool added = false;

	for (const QPair<QString, QString>& dev : m_dialog->getDeviceList())
	{
		// we set it as data, because the device list could get invalidated while the menu is up
		QAction* action;
		if (dev.first.compare(dev.second, Qt::CaseInsensitive) == 0)
			action = menu.addAction(dev.first);
		else
			action = menu.addAction(QStringLiteral("%1: %2").arg(dev.first).arg(dev.second));
		action->setData(dev.first);
		connect(action, &QAction::triggered, this, [this, action]() { doDeviceAutomaticBinding(action->data().toString()); });
		added = true;
	}

	if (!added)
	{
		QAction* action = menu.addAction(tr("No devices available"));
		action->setEnabled(false);
	}

	menu.exec(QCursor::pos());
}

void ControllerBindingWidget::onClearBindingsClicked()
{
	//: Binding: A pair of (host button, target button); Mapping: A list of bindings covering an entire controller. These are two different things (which might be the same in your language, please make sure to verify this).
	if (QMessageBox::question(QtUtils::GetRootWidget(this), tr("Clear Bindings"),
			//: Binding: A pair of (host button, target button); Mapping: A list of bindings covering an entire controller. These are two different things (which might be the same in your language, please make sure to verify this).
			tr("Are you sure you want to clear all bindings for this controller? This action cannot be undone.")) != QMessageBox::Yes)
	{
		return;
	}

	if (m_dialog->isEditingGlobalSettings())
	{
		{
			auto lock = Host::GetSettingsLock();
			Pad::ClearPortBindings(*Host::Internal::GetBaseSettingsLayer(), m_port_number);
		}
		Host::CommitBaseSettingChanges();
	}
	else
	{
		Pad::ClearPortBindings(*m_dialog->getProfileSettingsInterface(), m_port_number);
		QtHost::SaveGameSettings(m_dialog->getProfileSettingsInterface(), false);
	}

	// force a refresh after clearing
	g_emu_thread->applySettings();
	onTypeChanged();
}

void ControllerBindingWidget::doDeviceAutomaticBinding(const QString& device)
{
	std::vector<std::pair<GenericInputBinding, std::string>> mapping = InputManager::GetGenericBindingMapping(device.toStdString());
	if (mapping.empty())
	{
		QMessageBox::critical(QtUtils::GetRootWidget(this), tr("Automatic Binding"),
			tr("No generic bindings were generated for device '%1'. The controller/source may not support automatic mapping.").arg(device));
		return;
	}

	bool result;
	if (m_dialog->isEditingGlobalSettings())
	{
		{
			auto lock = Host::GetSettingsLock();
			result = Pad::MapController(*Host::Internal::GetBaseSettingsLayer(), m_port_number, mapping);
		}
		if (result)
			Host::CommitBaseSettingChanges();
	}
	else
	{
		result = Pad::MapController(*m_dialog->getProfileSettingsInterface(), m_port_number, mapping);
		if (result)
		{
			m_dialog->getProfileSettingsInterface()->Save();
			g_emu_thread->reloadInputBindings();
		}
	}

	// force a refresh after mapping
	if (result)
	{
		g_emu_thread->applySettings();
		onTypeChanged();
	}
}

//////////////////////////////////////////////////////////////////////////

ControllerMacroWidget::ControllerMacroWidget(ControllerBindingWidget* parent)
	: QWidget(parent)
{
	m_ui.setupUi(this);
	setWindowTitle(tr("Controller Port %1 Macros").arg(parent->getPortNumber() + 1u));
	createWidgets(parent);
}

ControllerMacroWidget::~ControllerMacroWidget() = default;

void ControllerMacroWidget::updateListItem(u32 index)
{
	//: This is the full text that appears in each option of the 16 available macros, and reads like this:\n\nMacro 1\nNot Configured/Buttons configured
	m_ui.portList->item(static_cast<int>(index))->setText(tr("Macro %1\n%2").arg(index + 1).arg(m_macros[index]->getSummary()));
}

void ControllerMacroWidget::createWidgets(ControllerBindingWidget* parent)
{
	for (u32 i = 0; i < NUM_MACROS; i++)
	{
		m_macros[i] = new ControllerMacroEditWidget(this, parent, i);
		m_ui.container->addWidget(m_macros[i]);

		QListWidgetItem* item = new QListWidgetItem();
		item->setIcon(QIcon::fromTheme(QStringLiteral("flashlight-line")));
		m_ui.portList->addItem(item);
		updateListItem(i);
	}

	m_ui.portList->setCurrentRow(0);
	m_ui.container->setCurrentIndex(0);

	connect(m_ui.portList, &QListWidget::currentRowChanged, m_ui.container, &QStackedWidget::setCurrentIndex);
}

//////////////////////////////////////////////////////////////////////////

ControllerMacroEditWidget::ControllerMacroEditWidget(ControllerMacroWidget* parent, ControllerBindingWidget* bwidget, u32 index)
	: QWidget(parent)
	, m_parent(parent)
	, m_bwidget(bwidget)
	, m_index(index)
{
	m_ui.setupUi(this);

	ControllerSettingsWindow* dialog = m_bwidget->getDialog();
	const std::string& section = m_bwidget->getConfigSection();
	const Pad::ControllerInfo* cinfo = Pad::GetControllerInfo(m_bwidget->getControllerType());
	if (!cinfo)
	{
		// Shouldn't ever happen.
		return;
	}

	// load binds (single string joined by &)
	const std::string binds_string(dialog->getStringValue(section.c_str(), TinyString::from_format("Macro{}Binds", index + 1u), ""));
	const std::vector<std::string_view> buttons_split(StringUtil::SplitString(binds_string, '&', true));

	for (const std::string_view& button : buttons_split)
	{
		for (const InputBindingInfo& bi : cinfo->bindings)
		{
			if (button == bi.name)
			{
				m_binds.push_back(&bi);
				break;
			}
		}
	}

	// populate list view
	for (const InputBindingInfo& bi : cinfo->bindings)
	{
		if (bi.bind_type == InputBindingInfo::Type::Motor)
			continue;

		QListWidgetItem* item = new QListWidgetItem();
		item->setText(qApp->translate("Pad", bi.display_name));
		item->setCheckState((std::find(m_binds.begin(), m_binds.end(), &bi) != m_binds.end()) ? Qt::Checked : Qt::Unchecked);
		m_ui.bindList->addItem(item);
	}

	ControllerSettingWidgetBinder::BindWidgetToInputProfileNormalized(
		dialog->getProfileSettingsInterface(), m_ui.pressure, section, fmt::format("Macro{}Pressure", index + 1u), 100.0f, 1.0f);
	ControllerSettingWidgetBinder::BindWidgetToInputProfileNormalized(
		dialog->getProfileSettingsInterface(), m_ui.deadzone, section, fmt::format("Macro{}Deadzone", index + 1u), 100.0f, 0.0f);
	connect(m_ui.pressure, &QSlider::valueChanged, this, &ControllerMacroEditWidget::onPressureChanged);
	connect(m_ui.deadzone, &QSlider::valueChanged, this, &ControllerMacroEditWidget::onDeadzoneChanged);
	onPressureChanged();
	onDeadzoneChanged();

	m_frequency = dialog->getIntValue(section.c_str(), fmt::format("Macro{}Frequency", index + 1u).c_str(), 0);
	updateFrequencyText();

	m_ui.trigger->initialize(
		dialog->getProfileSettingsInterface(), InputBindingInfo::Type::Macro, section, fmt::format("Macro{}", index + 1u));
	ControllerSettingWidgetBinder::BindWidgetToInputProfileBool(dialog->getProfileSettingsInterface(), m_ui.triggerToggle,
		section.c_str(), fmt::format("Macro{}Toggle", index + 1u), false);

	connect(m_ui.increaseFrequency, &QAbstractButton::clicked, this, [this]() { modFrequency(1); });
	connect(m_ui.decreateFrequency, &QAbstractButton::clicked, this, [this]() { modFrequency(-1); });
	connect(m_ui.setFrequency, &QAbstractButton::clicked, this, &ControllerMacroEditWidget::onSetFrequencyClicked);
	connect(m_ui.bindList, &QListWidget::itemChanged, this, &ControllerMacroEditWidget::updateBinds);
}

ControllerMacroEditWidget::~ControllerMacroEditWidget() = default;

QString ControllerMacroEditWidget::getSummary() const
{
	QString str;
	for (const InputBindingInfo* bi : m_binds)
	{
		if (!str.isEmpty())
			str += static_cast<QChar>('/');
		str += qApp->translate("Pad", bi->display_name);
	}
	return str.isEmpty() ? tr("Not Configured") : str;
}

void ControllerMacroEditWidget::onPressureChanged()
{
	m_ui.pressureValue->setText(tr("%1%").arg(m_ui.pressure->value()));
}

void ControllerMacroEditWidget::onDeadzoneChanged()
{
	m_ui.deadzoneValue->setText(tr("%1%").arg(m_ui.deadzone->value()));
}

void ControllerMacroEditWidget::onSetFrequencyClicked()
{
	bool okay;
	int new_freq = QInputDialog::getInt(
		this, tr("Set Frequency"), tr("Frequency: "), static_cast<int>(m_frequency), 0, std::numeric_limits<int>::max(), 1, &okay);
	if (!okay)
		return;

	m_frequency = static_cast<u32>(new_freq);
	updateFrequency();
}

void ControllerMacroEditWidget::modFrequency(s32 delta)
{
	if (delta < 0 && m_frequency == 0)
		return;

	m_frequency = static_cast<u32>(static_cast<s32>(m_frequency) + delta);
	updateFrequency();
}

void ControllerMacroEditWidget::updateFrequency()
{
	m_bwidget->getDialog()->setIntValue(
		m_bwidget->getConfigSection().c_str(), fmt::format("Macro{}Frequency", m_index + 1u).c_str(), static_cast<s32>(m_frequency));
	updateFrequencyText();
}

void ControllerMacroEditWidget::updateFrequencyText()
{
	if (m_frequency == 0)
		m_ui.frequencyText->setText(tr("Macro will not repeat."));
	else
		m_ui.frequencyText->setText(tr("Macro will toggle buttons every %1 frames.").arg(m_frequency));
}

void ControllerMacroEditWidget::updateBinds()
{
	ControllerSettingsWindow* dialog = m_bwidget->getDialog();
	const Pad::ControllerInfo* cinfo = Pad::GetControllerInfo(m_bwidget->getControllerType());
	if (!cinfo)
		return;

	std::vector<const InputBindingInfo*> new_binds;
	for (u32 i = 0, bind_index = 0; i < static_cast<u32>(cinfo->bindings.size()); i++)
	{
		const InputBindingInfo& bi = cinfo->bindings[i];
		if (bi.bind_type == InputBindingInfo::Type::Motor)
			continue;

		const QListWidgetItem* item = m_ui.bindList->item(static_cast<int>(bind_index));
		bind_index++;

		if (!item)
		{
			// shouldn't happen
			continue;
		}

		if (item->checkState() == Qt::Checked)
			new_binds.push_back(&bi);
	}
	if (m_binds == new_binds)
		return;

	m_binds = std::move(new_binds);

	std::string binds_string;
	for (const InputBindingInfo* bi : m_binds)
	{
		if (!binds_string.empty())
			binds_string.append(" & ");
		binds_string.append(bi->name);
	}

	const std::string& section = m_bwidget->getConfigSection();
	const std::string key(fmt::format("Macro{}Binds", m_index + 1u));
	if (binds_string.empty())
		dialog->clearSettingValue(section.c_str(), key.c_str());
	else
		dialog->setStringValue(section.c_str(), key.c_str(), binds_string.c_str());

	m_parent->updateListItem(m_index);
}

//////////////////////////////////////////////////////////////////////////

ControllerCustomSettingsWidget::ControllerCustomSettingsWidget(std::span<const SettingInfo> settings, std::string config_section,
	std::string config_prefix, const char* translation_ctx, ControllerSettingsWindow* dialog, QWidget* parent_widget)
	: QWidget(parent_widget)
	, m_settings(settings)
	, m_config_section(std::move(config_section))
	, m_config_prefix(std::move(config_prefix))
	, m_dialog(dialog)
{
	if (settings.empty())
		return;

	QScrollArea* sarea = new QScrollArea(this);
	QWidget* swidget = new QWidget(sarea);
	sarea->setWidget(swidget);
	sarea->setWidgetResizable(true);
	sarea->setFrameShape(QFrame::StyledPanel);
	sarea->setFrameShadow(QFrame::Sunken);

	QGridLayout* swidget_layout = new QGridLayout(swidget);
	createSettingWidgets(translation_ctx, swidget, swidget_layout);

	QVBoxLayout* layout = new QVBoxLayout(this);
	layout->setContentsMargins(0, 0, 0, 0);
	layout->addWidget(sarea);
}

ControllerCustomSettingsWidget::~ControllerCustomSettingsWidget() = default;

static std::tuple<QString, QString> getPrefixAndSuffixForIntFormat(const QString& format)
{
	QString prefix, suffix;
	QRegularExpression re(QStringLiteral("(.*)%.*d(.*)"));
	QRegularExpressionMatch match(re.match(format));
	if (match.isValid())
	{
		prefix = match.captured(1).replace(QStringLiteral("%%"), QStringLiteral("%"));
		suffix = match.captured(2).replace(QStringLiteral("%%"), QStringLiteral("%"));
	}

	return std::tie(prefix, suffix);
}

static std::tuple<QString, QString, int> getPrefixAndSuffixForFloatFormat(const QString& format)
{
	QString prefix, suffix;
	int decimals = -1;

	QRegularExpression re(QStringLiteral("(.*)%.*([0-9]+)f(.*)"));
	QRegularExpressionMatch match(re.match(format));
	if (match.isValid())
	{
		prefix = match.captured(1).replace(QStringLiteral("%%"), QStringLiteral("%"));
		suffix = match.captured(3).replace(QStringLiteral("%%"), QStringLiteral("%"));

		bool decimals_ok;
		decimals = match.captured(2).toInt(&decimals_ok);
		if (!decimals_ok)
			decimals = -1;
	}
	else
	{
		re = QRegularExpression(QStringLiteral("(.*)%.*f(.*)"));
		match = re.match(format);
		prefix = match.captured(1).replace(QStringLiteral("%%"), QStringLiteral("%"));
		suffix = match.captured(2).replace(QStringLiteral("%%"), QStringLiteral("%"));
	}

	return std::tie(prefix, suffix, decimals);
}


namespace
{
class UePcbDescriptionFilter final : public QObject
{
public:
	UePcbDescriptionFilter(QLabel* target, const QString& default_text, QObject* parent)
		: QObject(parent), m_target(target), m_default_text(default_text) {}

protected:
	bool eventFilter(QObject* watched, QEvent* event) override
	{
		if (event->type() == QEvent::Enter)
		{
			const QString text = watched->property("uepcb_description").toString();
			if (!text.isEmpty())
				m_target->setText(text);
		}
		else if (event->type() == QEvent::Leave)
		{
			m_target->setText(m_default_text);
		}
		return QObject::eventFilter(watched, event);
	}

private:
	QLabel* m_target;
	QString m_default_text;
};
}

void ControllerCustomSettingsWidget::createSettingWidgets(const char* translation_ctx, QWidget* widget_parent, QGridLayout* layout)
{
	SettingsInterface* sif = m_dialog->getProfileSettingsInterface();
	int current_row = 0;

	// UE PCB 1.6.4B compact TCP/UDP + multi-instance endpoint layout. Detailed help is shown in one
	// fixed description area at the bottom when the pointer enters a setting.
	if (m_config_prefix == "UePcb_")
	{
		layout->setColumnStretch(1, 1);
		layout->setColumnStretch(3, 1);
		layout->setHorizontalSpacing(12);
		layout->setVerticalSpacing(5);

		const auto addString = [&](const char* key, const QString& label, const char* def, int row, int col) {
			QLabel* l = new QLabel(label, widget_parent);
			QLineEdit* le = new QLineEdit(widget_parent);
			le->setObjectName(QString::fromUtf8(key));
			ControllerSettingWidgetBinder::BindWidgetToInputProfileString(sif, le, m_config_section, m_config_prefix + key, def);
			layout->addWidget(l, row, col); layout->addWidget(le, row, col + 1);
			return std::pair<QLabel*, QLineEdit*>(l, le);
		};
		const auto addInt = [&](const char* key, const QString& label, int def, int minv, int maxv, int step, int row, int col) {
			QLabel* l = new QLabel(label, widget_parent);
			QSpinBox* sb = new QSpinBox(widget_parent);
			sb->setObjectName(QString::fromUtf8(key)); sb->setRange(minv, maxv); sb->setSingleStep(step);
			ControllerSettingWidgetBinder::BindWidgetToInputProfileInt(sif, sb, m_config_section, m_config_prefix + key, def);
			layout->addWidget(l, row, col); layout->addWidget(sb, row, col + 1);
			return std::pair<QLabel*, QSpinBox*>(l, sb);
		};
		const auto addEditableIPCombo = [&](const char* key, const QString& label, const char* def, int row, int col) {
			QLabel* l = new QLabel(label, widget_parent);
			QComboBox* cb = new QComboBox(widget_parent);
			cb->setEditable(true);
			cb->setInsertPolicy(QComboBox::NoInsert);
			cb->setObjectName(QString::fromUtf8(key) + QStringLiteral("Combo"));
			for (int i = 1; i <= 10; i++)
			{
				const std::string history_key = m_config_prefix + fmt::format("History{}", i);
				const std::string ip = m_dialog->getStringValue(m_config_section.c_str(), history_key.c_str(), "");
				if (!ip.empty() && cb->findText(QString::fromStdString(ip)) < 0)
					cb->addItem(QString::fromStdString(ip));
			}
			cb->lineEdit()->setObjectName(QString::fromUtf8(key));
			ControllerSettingWidgetBinder::BindWidgetToInputProfileString(
				sif, cb->lineEdit(), m_config_section, m_config_prefix + key, def);
			layout->addWidget(l, row, col); layout->addWidget(cb, row, col + 1);
			return std::pair<QLabel*, QComboBox*>(l, cb);
		};

		QLabel* modeLabel = new QLabel(tr("Connection Mode"), widget_parent);
		QComboBox* mode = new QComboBox(widget_parent); mode->setObjectName(QStringLiteral("ConnectionMode"));
		mode->addItem(tr("UDP")); mode->addItem(tr("TCP"));
		ControllerSettingWidgetBinder::BindWidgetToInputProfileInt(sif, mode, m_config_section, m_config_prefix + "ConnectionMode", 0, 0);
		layout->addWidget(modeLabel, current_row, 0); layout->addWidget(mode, current_row, 1);
		auto portPair = addInt("Port", tr("Local UDP Port"), 7500, 1, 65535, 1, current_row, 2); current_row++;

		// 1.6.4B: editable IP history and an explicit destination port form one
		// complete UDP endpoint. Saved IPs are now selected directly inside the
		// IP field, removing the old Manual-vs-Saved ambiguity.
		auto peer1 = addEditableIPCombo("Peer1IP", tr("Direct Peer 1 IP"), "", current_row, 0);
		auto peer1port = addInt("Peer1Port", tr("Port"), 7500, 1, 65535, 1, current_row, 2); current_row++;
		auto peer2 = addEditableIPCombo("Peer2IP", tr("Direct Peer 2 IP"), "", current_row, 0);
		auto peer2port = addInt("Peer2Port", tr("Port"), 7500, 1, 65535, 1, current_row, 2); current_row++;
		auto peer3 = addEditableIPCombo("Peer3IP", tr("Direct Peer 3 IP"), "", current_row, 0);
		auto peer3port = addInt("Peer3Port", tr("Port"), 7500, 1, 65535, 1, current_row, 2); current_row++;

		QLabel* roleLabel = new QLabel(tr("TCP Role"), widget_parent);
		QComboBox* role = new QComboBox(widget_parent); role->setObjectName(QStringLiteral("TCPRole"));
		role->addItem(tr("Host")); role->addItem(tr("Client"));
		ControllerSettingWidgetBinder::BindWidgetToInputProfileInt(sif, role, m_config_section, m_config_prefix + "TCPRole", 0, 0);
		layout->addWidget(roleLabel, current_row, 0); layout->addWidget(role, current_row, 1);
		auto bindPair = addString("TCPBindIP", tr("Listen IP"), "0.0.0.0", current_row, 2); current_row++;
		auto hostPair = addEditableIPCombo("TCPHostIP", tr("TCP Host IP"), "127.0.0.1", current_row, 0); current_row++;

		// Save/Clear are one-shot actions, so push buttons are clearer than
		// checkboxes (there is no persistent on/off state to display).
		QPushButton* remember = new QPushButton(tr("Save Current IPs"), widget_parent);
		remember->setObjectName(QStringLiteral("RememberCurrentIPs"));
		layout->addWidget(remember, current_row, 0, 1, 2);
		QPushButton* clear = new QPushButton(tr("Clear Saved IPs"), widget_parent);
		clear->setObjectName(QStringLiteral("ClearIPHistory"));
		layout->addWidget(clear, current_row, 2, 1, 2); current_row++;

		QCheckBox* advanced = new QCheckBox(tr("Advanced Settings"), widget_parent);
		advanced->setObjectName(QStringLiteral("AdvancedSettings"));
		ControllerSettingWidgetBinder::BindWidgetToInputProfileBool(sif, advanced, m_config_section, m_config_prefix + "AdvancedSettings", false);
		layout->addWidget(advanced, current_row++, 0, 1, 4);
		auto grace = addInt("JitterGraceMs", tr("Jitter Grace (ms)"), 4, 0, 20, 1, current_row, 0);
		auto decay = addInt("JitterDecayMs", tr("Buffer Decay (ms)"), 250, 250, 60000, 250, current_row, 2); current_row++;
		auto minTarget = addInt("JitterMinTarget", tr("Minimum Target Buffer"), 1, 1, 8, 1, current_row, 0);
		auto maxTarget = addInt("JitterMaxTarget", tr("Maximum Target Buffer"), 8, 1, 8, 1, current_row, 2); current_row++;
		auto maxQueue = addInt("JitterMaxPackets", tr("Maximum Jitter Queue"), 8, 1, 32, 1, current_row, 0);
		auto broadcast = addString("TargetIP", tr("Broadcast Address"), "255.255.255.255", current_row, 2); current_row++;

		// 1.6.4B Sync Priority controls. Sync Hold is the recommended baseline;
		// the remaining switches stay independent for A/B testing.
		// Keep both primary timing controls on one compact row. The units live
		// inside the spin boxes; detailed meanings are in the hover Description.
		QCheckBox* betaSyncHold = new QCheckBox(tr("Sync Hold"), widget_parent);
		ControllerSettingWidgetBinder::BindWidgetToInputProfileBool(
			sif, betaSyncHold, m_config_section, m_config_prefix + "BetaSyncHold", true);
		QSpinBox* betaHoldMs = new QSpinBox(widget_parent);
		betaHoldMs->setObjectName(QStringLiteral("BetaSyncHoldMs"));
		betaHoldMs->setRange(3, 100);
		betaHoldMs->setSingleStep(1);
		betaHoldMs->setSuffix(tr(" ms"));
		ControllerSettingWidgetBinder::BindWidgetToInputProfileInt(
			sif, betaHoldMs, m_config_section, m_config_prefix + "BetaSyncHoldMs", 30);

		QCheckBox* betaPlayout = new QCheckBox(tr("Adaptive Playout"), widget_parent);
		ControllerSettingWidgetBinder::BindWidgetToInputProfileBool(
			sif, betaPlayout, m_config_section, m_config_prefix + "BetaAdaptivePlayout", false);
		QSpinBox* betaPlayoutMax = new QSpinBox(widget_parent);
		betaPlayoutMax->setObjectName(QStringLiteral("BetaPlayoutMaxMs"));
		betaPlayoutMax->setRange(0, 20);
		betaPlayoutMax->setSingleStep(1);
		betaPlayoutMax->setSuffix(tr(" ms"));
		ControllerSettingWidgetBinder::BindWidgetToInputProfileInt(
			sif, betaPlayoutMax, m_config_section, m_config_prefix + "BetaPlayoutMaxMs", 3);

		layout->addWidget(betaSyncHold, current_row, 0);
		layout->addWidget(betaHoldMs, current_row, 1);
		layout->addWidget(betaPlayout, current_row, 2);
		layout->addWidget(betaPlayoutMax, current_row, 3);
		current_row++;

		QCheckBox* betaRetransmit = new QCheckBox(tr("UDP Retransmit"), widget_parent);
		ControllerSettingWidgetBinder::BindWidgetToInputProfileBool(
			sif, betaRetransmit, m_config_section, m_config_prefix + "BetaUdpRetransmit", false);
		layout->addWidget(betaRetransmit, current_row, 0, 1, 2);
		QCheckBox* betaStallGuard = new QCheckBox(tr("Global Stall Guard"), widget_parent);
		ControllerSettingWidgetBinder::BindWidgetToInputProfileBool(
			sif, betaStallGuard, m_config_section, m_config_prefix + "BetaGlobalStallGuard", false);
		layout->addWidget(betaStallGuard, current_row, 2, 1, 2); current_row++;

		auto mac = addString("MacHex", tr("MAC 12-hex"), "", current_row, 0); current_row++;

		// Fixed-height help area: detailed text moves here instead of consuming rows.
		QLabel* descriptionTitle = new QLabel(tr("Description"), widget_parent);
		QFont titleFont = descriptionTitle->font(); titleFont.setBold(true); descriptionTitle->setFont(titleFont);
		layout->addWidget(descriptionTitle, current_row++, 0, 1, 4);
		const QString defaultHelp = tr("Move the mouse over a UE PCB setting to see detailed information here.");
		QLabel* description = new QLabel(defaultHelp, widget_parent);
		description->setWordWrap(true); description->setAlignment(Qt::AlignLeft | Qt::AlignTop);
		description->setMinimumHeight(description->fontMetrics().lineSpacing() * 4 + 12);
		description->setMaximumHeight(description->fontMetrics().lineSpacing() * 5 + 16);
		layout->addWidget(description, current_row++, 0, 1, 4);
		auto* helpFilter = new UePcbDescriptionFilter(description, defaultHelp, widget_parent);
		const auto help = [&](QWidget* w, const QString& text) { w->setProperty("uepcb_description", text); w->installEventFilter(helpFilter); };
		const auto helpPair = [&](auto pair, const QString& text) { help(pair.first, text); help(pair.second, text); };

		const auto helpEditablePair = [&](auto pair, const QString& text) {
			help(pair.first, text);
			help(pair.second, text);
			if (pair.second->lineEdit())
				help(pair.second->lineEdit(), text);
		};

		const QString modeHelp = tr("UDP is the recommended low-latency mode. In UDP, Local UDP Port is this emulator instance's receive port; each Direct Peer uses its own IP + destination Port. Same-PC multi-instance play must give every emulator a unique Local UDP Port, for example 7500, 7501, 7502 and 7503. TCP keeps the existing Host/Client hub and uses the main port as its TCP connection port.");
		help(modeLabel, modeHelp); help(mode, modeHelp); helpPair(portPair, modeHelp);
		const QString udpHelp = tr("UDP Direct Peer routing uses a complete endpoint: IP + Port. The IP field is editable, and its drop-down directly lists the shared saved-IP history; selecting an item simply fills that IP into the same field. For a 4-player game, enter the other three emulator endpoints and do not enter this instance's own endpoint. If all three peer IPs are empty, UEPCB falls back to Broadcast Address on the Local UDP Port.");
		helpEditablePair(peer1, udpHelp); helpPair(peer1port, udpHelp);
		helpEditablePair(peer2, udpHelp); helpPair(peer2port, udpHelp);
		helpEditablePair(peer3, udpHelp); helpPair(peer3port, udpHelp);
		const QString tcpRoleHelp = tr("TCP Host accepts the other players and relays Ethernet frames. TCP Client connects to that Host. TCP uses an N-way Host hub and TCP_NODELAY and does not use the UDP adaptive jitter buffer.");
		help(roleLabel, tcpRoleHelp); help(role, tcpRoleHelp);
		helpPair(bindPair, tr("TCP Host Listen IP: 0.0.0.0 accepts connections on all LAN/VPN adapters. Use 127.0.0.1 only when every instance is on the same PC. This field is ignored in TCP Client mode."));
		helpEditablePair(hostPair, tr("TCP Client Host IP is now also an editable saved-IP drop-down. Type an address directly or select one from the shared history. Use 127.0.0.1 for same-PC TCP; across LAN or VPN, use the Host machine's LAN/Tailscale/VPN address."));
		const QString rememberHelp = tr("Save Current IPs stores the current Peer 1-3 IP fields, or the TCP Client Host IP, into the shared 10-entry history immediately. Recent addresses move to the front and duplicates are removed. Ports are not stored in IP history.");
		help(remember, rememberHelp);
		help(clear, tr("Clear Saved IPs erases all ten saved addresses and also clears the current UDP Peer IP and TCP Host IP fields. Peer port values are left unchanged."));
		help(advanced, tr("Leave Advanced Settings unchecked for the 1.6.4B Fast Attack defaults: Jitter Grace 4 ms, Buffer Decay 250 ms, Minimum Target 1, Maximum Target 8 and Queue 8. These values are tuned for fast reaction to real packet disorder and a smooth recovery from Target 8 back to 1 in about 1.75 seconds. Enable Advanced Settings only when manually testing other values. TCP bypasses these controls."));
		helpPair(grace, tr("Jitter Grace is the quiet window for ordinary short UDP reordering. In 1.6.4B, Fast Attack does not raise the target inside this window. Default: 4 ms. If the gap survives beyond Grace, the target may jump according to gap age and ahead depth before any forced skip occurs."));
		helpPair(decay, tr("Buffer Decay is the stable playback time before the adaptive target drops by one level. Default: 250 ms. With Maximum Target 8, recovery 8 -> 1 takes about 1.75 seconds, giving the intended slow/smooth recovery after a fast rise."));
		helpPair(minTarget, tr("Minimum Target Buffer sets the lowest adaptive UDP playback depth. Default: 1."));
		helpPair(maxTarget, tr("Maximum Target Buffer limits Fast Attack growth. Default: 8. A real gap can jump directly to 2/3/5/6/8 depending on how long the expected packet is missing and how many later packets have already arrived; high ping alone does not raise it."));
		helpPair(maxQueue, tr("Maximum Jitter Queue is the hard per-peer UDP holding limit. Default: 8, matching the new Maximum Target. Keep it small; larger queues can accumulate unnecessary latency."));
		helpPair(broadcast, tr("Broadcast Address is used only in UDP when all three Direct Peer IPs are empty. Default: 255.255.255.255. Broadcast uses this instance's Local UDP Port; for reliable same-PC multi-instance routing, use explicit Direct Peer IP + Port endpoints instead."));
		const QString syncHoldHelp = tr("Sync Hold is the recommended synchronization-first option and is ON by default for new/reset 1.6.4B profiles. When a UDP sequence is missing, UEPCB waits up to 30 ms instead of immediately forced-skipping. Fast Attack can raise the target during that wait, so UEPCB reacts to real jitter/bursts without increasing Hold merely because ping is high.");
		help(betaSyncHold, syncHoldHelp);
		help(betaHoldMs, syncHoldHelp);
		const QString playoutHelp = tr("Adaptive Playout is optional. In 1.6.4B the target can now rise through Fast Attack before a forced skip, so Playout may become active earlier when the target is elevated. It still uses 1 ms per buffer step with a 3 ms ceiling. Keep it OFF for the clean Sync Hold baseline, or enable it only for A/B testing because the observed hold can exceed the target until the next USB poll.");
		help(betaPlayout, playoutHelp);
		help(betaPlayoutMax, playoutHelp);
		help(betaRetransmit, tr("UDP Retransmit remains experimental and is not recommended for normal stable links. It waits until a real gap survives 18 ms, sends one peer-directed NACK, and only that recovery path may extend the wait to about 60 ms. Normal Sync Hold remains 30 ms; 1.6.4B does not use a 120 ms gameplay hold."));
		help(betaStallGuard, tr("Global Stall Guard remains an optional diagnostic/protection feature. If different peers forced-skip within 100 ms, that pattern can indicate a local PCSX2/USB scheduling stall rather than independent network loss; the guard prevents several peer buffers from staying enlarged. It has not yet been validated as strongly as Sync Hold, so leave it off for the normal baseline."));
		helpPair(mac, tr("Leave MAC 12-hex blank so each emulator instance automatically generates a unique Namco MAC. Only enter 12 hexadecimal digits when a fixed MAC is specifically required."));

		// 1.6.4B: refresh the saved-IP choices without changing the IP currently
		// typed/selected in each editable combo.
		const auto refreshIPCombos = [=]() {
			for (QComboBox* cb : {peer1.second, peer2.second, peer3.second, hostPair.second})
			{
				const QString current = cb->currentText();
				QLineEdit* edit = cb->lineEdit();
				const bool old_cb_block = cb->blockSignals(true);
				const bool old_edit_block = edit ? edit->blockSignals(true) : false;
				cb->clear();
				for (int i = 1; i <= 10; i++)
				{
					const std::string key = m_config_prefix + fmt::format("History{}", i);
					const QString ip = QString::fromStdString(m_dialog->getStringValue(m_config_section.c_str(), key.c_str(), "")).trimmed();
					if (!ip.isEmpty() && cb->findText(ip) < 0)
						cb->addItem(ip);
				}
				cb->setEditText(current);
				if (edit)
					edit->blockSignals(old_edit_block);
				cb->blockSignals(old_cb_block);
			}
		};
		connect(remember, &QPushButton::clicked, this, [=]() {
			QStringList candidates;
			if (mode->currentIndex() == 1 && role->currentIndex() == 1)
				candidates << hostPair.second->currentText().trimmed();
			else if (mode->currentIndex() == 0)
				candidates << peer1.second->currentText().trimmed() << peer2.second->currentText().trimmed() << peer3.second->currentText().trimmed();
			for (int i = 1; i <= 10; i++)
			{
				const std::string key = m_config_prefix + fmt::format("History{}", i);
				candidates << QString::fromStdString(m_dialog->getStringValue(m_config_section.c_str(), key.c_str(), ""));
			}
			QStringList unique;
			for (const QString& ip : candidates)
				if (!ip.trimmed().isEmpty() && !unique.contains(ip.trimmed()))
					unique << ip.trimmed();
			for (int i = 1; i <= 10; i++)
			{
				const std::string value = (i <= unique.size()) ? unique[i - 1].toStdString() : std::string();
				m_dialog->setStringValue(
					m_config_section.c_str(),
					(m_config_prefix + fmt::format("History{}", i)).c_str(),
					value.c_str());
			}
			refreshIPCombos();
		});
		connect(clear, &QPushButton::clicked, this, [=]() {
			for (int i = 1; i <= 10; i++)
				m_dialog->setStringValue(m_config_section.c_str(), (m_config_prefix + fmt::format("History{}", i)).c_str(), "");

			// The user explicitly prefers Clear to remove the active IP entries too.
			for (const char* key : {"Peer1IP", "Peer2IP", "Peer3IP", "TCPHostIP"})
				m_dialog->setStringValue(m_config_section.c_str(), (m_config_prefix + key).c_str(), "");
			for (const char* key : {"Peer1HistorySlot", "Peer2HistorySlot", "Peer3HistorySlot", "TCPHostHistorySlot"})
				m_dialog->setIntValue(m_config_section.c_str(), (m_config_prefix + key).c_str(), 0);

			peer1.second->setEditText(QString());
			peer2.second->setEditText(QString());
			peer3.second->setEditText(QString());
			hostPair.second->setEditText(QString());
			refreshIPCombos();
		});

		const auto setPairEnabled = [](auto pair, bool enabled) { pair.first->setEnabled(enabled); pair.second->setEnabled(enabled); };
		const auto updateEnabled = [=]() {
			const bool isTcp = (mode->currentIndex() == 1);
			const bool isHost = (role->currentIndex() == 0);
			const bool adv = advanced->isChecked();

			portPair.first->setText(isTcp ? tr("TCP Port") : tr("Local UDP Port"));
			setPairEnabled(peer1, !isTcp); setPairEnabled(peer1port, !isTcp);
			setPairEnabled(peer2, !isTcp); setPairEnabled(peer2port, !isTcp);
			setPairEnabled(peer3, !isTcp); setPairEnabled(peer3port, !isTcp);
			roleLabel->setEnabled(isTcp); role->setEnabled(isTcp);
			setPairEnabled(bindPair, isTcp && isHost);
			setPairEnabled(hostPair, isTcp && !isHost);

			const bool jitterEnabled = !isTcp && adv;
			setPairEnabled(grace, jitterEnabled);
			setPairEnabled(decay, jitterEnabled);
			setPairEnabled(minTarget, jitterEnabled);
			setPairEnabled(maxTarget, jitterEnabled);
			setPairEnabled(maxQueue, jitterEnabled);
			setPairEnabled(broadcast, jitterEnabled);
			advanced->setEnabled(!isTcp);

			// Beta features are UDP-only. Their switches remain independent,
			// while numeric fields only unlock when the matching switch is on.
			betaSyncHold->setEnabled(!isTcp);
			betaRetransmit->setEnabled(!isTcp);
			betaPlayout->setEnabled(!isTcp);
			betaStallGuard->setEnabled(!isTcp);
			betaHoldMs->setEnabled(!isTcp && betaSyncHold->isChecked());
			betaPlayoutMax->setEnabled(!isTcp && betaPlayout->isChecked());
		};
		connect(mode, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int) { updateEnabled(); });
		connect(role, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int) { updateEnabled(); });
		connect(advanced, &QCheckBox::toggled, this, [=](bool) { updateEnabled(); });
		connect(betaSyncHold, &QCheckBox::toggled, this, [=](bool) { updateEnabled(); });
		connect(betaPlayout, &QCheckBox::toggled, this, [=](bool) { updateEnabled(); });
		updateEnabled();

		QHBoxLayout* bottom_hlayout = new QHBoxLayout();
		QPushButton* restore_defaults = new QPushButton(tr("Restore Default Settings"), this);
		restore_defaults->setIcon(QIcon::fromTheme(QStringLiteral("restart-line")));
		connect(restore_defaults, &QPushButton::clicked, this, &ControllerCustomSettingsWidget::restoreDefaults);
		bottom_hlayout->addStretch(1); bottom_hlayout->addWidget(restore_defaults);
		layout->addLayout(bottom_hlayout, current_row++, 0, 1, 4);
		layout->addItem(new QSpacerItem(1, 1, QSizePolicy::Minimum, QSizePolicy::Expanding), current_row++, 0, 1, 4);
		return;
	}

	for (const SettingInfo& si : m_settings)
	{
		std::string key_name = m_config_prefix + si.name;

		switch (si.type)
		{
			case SettingInfo::Type::Boolean:
			{
				QCheckBox* cb = new QCheckBox(qApp->translate(translation_ctx, si.display_name), widget_parent);
				cb->setObjectName(QString::fromUtf8(si.name));
				ControllerSettingWidgetBinder::BindWidgetToInputProfileBool(
					sif, cb, m_config_section, std::move(key_name), si.BooleanDefaultValue());
				layout->addWidget(cb, current_row, 0, 1, 4);
				current_row++;
			}
			break;

			case SettingInfo::Type::Integer:
			{
				QSpinBox* sb = new QSpinBox(widget_parent);
				sb->setObjectName(QString::fromUtf8(si.name));
				sb->setMinimum(si.IntegerMinValue());
				sb->setMaximum(si.IntegerMaxValue());
				sb->setSingleStep(si.IntegerStepValue());
				if (si.format)
				{
					const auto [prefix, suffix] = getPrefixAndSuffixForIntFormat(qApp->translate(translation_ctx, si.format));
					sb->setPrefix(prefix);
					sb->setSuffix(suffix);
				}
				ControllerSettingWidgetBinder::BindWidgetToInputProfileInt(
					sif, sb, m_config_section, std::move(key_name), si.IntegerDefaultValue());
				layout->addWidget(new QLabel(qApp->translate(translation_ctx, si.display_name), widget_parent), current_row, 0);
				layout->addWidget(sb, current_row, 1, 1, 3);
				current_row++;
			}
			break;

			case SettingInfo::Type::IntegerList:
			{
				QComboBox* cb = new QComboBox(widget_parent);
				cb->setObjectName(QString::fromUtf8(si.name));
				for (u32 i = 0; si.options[i] != nullptr; i++)
					cb->addItem(qApp->translate(translation_ctx, si.options[i]));
				ControllerSettingWidgetBinder::BindWidgetToInputProfileInt(
					sif, cb, m_config_section, std::move(key_name), si.IntegerDefaultValue(), si.IntegerMinValue());
				layout->addWidget(new QLabel(qApp->translate(translation_ctx, si.display_name), widget_parent), current_row, 0);
				layout->addWidget(cb, current_row, 1, 1, 3);
				current_row++;
			}
			break;

			case SettingInfo::Type::Float:
			{
				QDoubleSpinBox* sb = new QDoubleSpinBox(widget_parent);
				sb->setObjectName(QString::fromUtf8(si.name));
				sb->setMinimum(si.FloatMinValue() * si.multiplier);
				sb->setMaximum(si.FloatMaxValue() * si.multiplier);
				sb->setSingleStep(si.FloatStepValue() * si.multiplier);

				if (si.format)
				{
					const auto [prefix, suffix, decimals] = getPrefixAndSuffixForFloatFormat(qApp->translate(translation_ctx, si.format));
					sb->setPrefix(prefix);
					if (decimals >= 0)
						sb->setDecimals(decimals);
					sb->setSuffix(suffix);
				}

				ControllerSettingWidgetBinder::BindWidgetToInputProfileFloat(
					sif, sb, m_config_section, std::move(key_name), si.FloatDefaultValue(), si.multiplier);
				layout->addWidget(new QLabel(qApp->translate(translation_ctx, si.display_name), widget_parent), current_row, 0);
				layout->addWidget(sb, current_row, 1, 1, 3);
				current_row++;
			}
			break;

			case SettingInfo::Type::String:
			{
				QLineEdit* le = new QLineEdit(widget_parent);
				le->setObjectName(QString::fromUtf8(si.name));
				ControllerSettingWidgetBinder::BindWidgetToInputProfileString(
					sif, le, m_config_section, std::move(key_name), si.StringDefaultValue());
				layout->addWidget(new QLabel(qApp->translate(translation_ctx, si.display_name), widget_parent), current_row, 0);
				layout->addWidget(le, current_row, 1, 1, 3);
				current_row++;
			}
			break;

			case SettingInfo::Type::StringList:
			{
				QComboBox* cb = new QComboBox(widget_parent);
				cb->setObjectName(QString::fromUtf8(si.name));
				if (si.get_options)
				{
					std::vector<std::pair<std::string, std::string>> options(si.get_options());
					for (const auto& [name, display_name] : options)
						cb->addItem(QString::fromStdString(display_name), QString::fromStdString(name));
				}
				else if (si.options)
				{
					for (u32 i = 0; si.options[i] != nullptr; i++)
						cb->addItem(qApp->translate(translation_ctx, si.options[i]), QString::fromUtf8(si.options[i]));
				}
				ControllerSettingWidgetBinder::BindWidgetToInputProfileString(
					sif, cb, m_config_section, std::move(key_name), si.StringDefaultValue());
				layout->addWidget(new QLabel(qApp->translate(translation_ctx, si.display_name), widget_parent), current_row, 0);
				layout->addWidget(cb, current_row, 1, 1, 3);
				current_row++;
			}
			break;

			case SettingInfo::Type::Path:
			{
				QLineEdit* le = new QLineEdit(widget_parent);
				le->setObjectName(QString::fromUtf8(si.name));
				QPushButton* browse_button = new QPushButton(tr("Browse..."), widget_parent);
				ControllerSettingWidgetBinder::BindWidgetToInputProfileString(
					sif, le, m_config_section, std::move(key_name), si.StringDefaultValue());
				connect(browse_button, &QPushButton::clicked, [this, le]() {
					const QString path(QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, tr("Select File"))));
					if (!path.isEmpty())
						le->setText(path);
				});

				QHBoxLayout* hbox = new QHBoxLayout();
				hbox->addWidget(le, 1);
				hbox->addWidget(browse_button);

				layout->addWidget(new QLabel(qApp->translate(translation_ctx, si.display_name), widget_parent), current_row, 0);
				layout->addLayout(hbox, current_row, 1, 1, 3);
				current_row++;
			}
			break;
		}

		QLabel* label = new QLabel(si.description ? qApp->translate(translation_ctx, si.description) : QString(), widget_parent);
		label->setWordWrap(true);
		layout->addWidget(label, current_row++, 0, 1, 4);

		layout->addItem(new QSpacerItem(1, 10, QSizePolicy::Minimum, QSizePolicy::Fixed), current_row++, 0, 1, 4);
	}

	QHBoxLayout* bottom_hlayout = new QHBoxLayout();
	QPushButton* restore_defaults = new QPushButton(tr("Restore Default Settings"), this);
	restore_defaults->setIcon(QIcon::fromTheme(QStringLiteral("restart-line")));
	connect(restore_defaults, &QPushButton::clicked, this, &ControllerCustomSettingsWidget::restoreDefaults);
	bottom_hlayout->addStretch(1);
	bottom_hlayout->addWidget(restore_defaults);
	layout->addLayout(bottom_hlayout, current_row++, 0, 1, 4);

	layout->addItem(new QSpacerItem(1, 1, QSizePolicy::Minimum, QSizePolicy::Expanding), current_row++, 0, 1, 4);
}

void ControllerCustomSettingsWidget::restoreDefaults()
{
	for (const SettingInfo& si : m_settings)
	{
		const QString key(QString::fromStdString(si.name));

		switch (si.type)
		{
			case SettingInfo::Type::Boolean:
			{
				QCheckBox* widget = findChild<QCheckBox*>(QString::fromStdString(si.name));
				if (widget)
					widget->setChecked(si.BooleanDefaultValue());
			}
			break;

			case SettingInfo::Type::Integer:
			{
				QSpinBox* widget = findChild<QSpinBox*>(QString::fromStdString(si.name));
				if (widget)
					widget->setValue(si.IntegerDefaultValue());
				else if (QComboBox* combo = findChild<QComboBox*>(QString::fromStdString(si.name)))
					combo->setCurrentIndex(si.IntegerDefaultValue());
			}
			break;

			case SettingInfo::Type::IntegerList:
			{
				QComboBox* widget = findChild<QComboBox*>(QString::fromStdString(si.name));
				if (widget)
					widget->setCurrentIndex(si.IntegerDefaultValue() - si.IntegerMinValue());
			}
			break;

			case SettingInfo::Type::Float:
			{
				QDoubleSpinBox* widget = findChild<QDoubleSpinBox*>(QString::fromStdString(si.name));
				if (widget)
					widget->setValue(si.FloatDefaultValue() * si.multiplier);
			}
			break;

			case SettingInfo::Type::String:
			{
				QLineEdit* widget = findChild<QLineEdit*>(QString::fromStdString(si.name));
				if (widget)
					widget->setText(QString::fromUtf8(si.StringDefaultValue()));
			}
			break;

			case SettingInfo::Type::StringList:
			{
				QComboBox* widget = findChild<QComboBox*>(QString::fromStdString(si.name));
				if (widget)
				{
					const QString default_value(QString::fromUtf8(si.StringDefaultValue()));
					int index = widget->findData(default_value);
					if (index < 0)
						index = widget->findText(default_value);
					if (index >= 0)
						widget->setCurrentIndex(index);
				}
			}
			break;

			case SettingInfo::Type::Path:
			{
				QLineEdit* widget = findChild<QLineEdit*>(QString::fromStdString(si.name));
				if (widget)
					widget->setText(QString::fromUtf8(si.StringDefaultValue()));
			}
			break;
		}
	}
}


//////////////////////////////////////////////////////////////////////////

ControllerBindingWidget_Base::ControllerBindingWidget_Base(ControllerBindingWidget* parent)
	: QWidget(parent)
{
}

ControllerBindingWidget_Base::~ControllerBindingWidget_Base()
{
}

QIcon ControllerBindingWidget_Base::getIcon() const
{
	return QIcon::fromTheme("controller-strike-line");
}

void ControllerBindingWidget_Base::initBindingWidgets()
{
	const Pad::ControllerInfo* cinfo = Pad::GetControllerInfo(getControllerType());
	if (!cinfo)
		return;

	const std::string& config_section = getConfigSection();
	SettingsInterface* sif = getDialog()->getProfileSettingsInterface();

	for (const InputBindingInfo& bi : cinfo->bindings)
	{
		if (bi.bind_type == InputBindingInfo::Type::Axis || bi.bind_type == InputBindingInfo::Type::HalfAxis ||
			bi.bind_type == InputBindingInfo::Type::Button || bi.bind_type == InputBindingInfo::Type::Pointer ||
			bi.bind_type == InputBindingInfo::Type::Device)
		{
			InputBindingWidget* widget = findChild<InputBindingWidget*>(QString::fromStdString(bi.name));
			if (!widget)
			{
				Console.Error("(ControllerBindingWidget_Base) No widget found for '%s' (%s)", bi.name, cinfo->name);
				continue;
			}

			widget->initialize(sif, bi.bind_type, config_section, bi.name);
		}
	}

	switch (cinfo->vibration_caps)
	{
		case Pad::VibrationCapabilities::LargeSmallMotors:
		{
			InputVibrationBindingWidget* widget = findChild<InputVibrationBindingWidget*>(QStringLiteral("LargeMotor"));
			if (widget)
				widget->setKey(getDialog(), config_section, "LargeMotor");

			widget = findChild<InputVibrationBindingWidget*>(QStringLiteral("SmallMotor"));
			if (widget)
				widget->setKey(getDialog(), config_section, "SmallMotor");
		}
		break;

		case Pad::VibrationCapabilities::SingleMotor:
		{
			InputVibrationBindingWidget* widget = findChild<InputVibrationBindingWidget*>(QStringLiteral("Motor"));
			if (widget)
				widget->setKey(getDialog(), config_section, "Motor");
		}
		break;

		case Pad::VibrationCapabilities::NoVibration:
		default:
			break;
	}
}

ControllerBindingWidget_DualShock2::ControllerBindingWidget_DualShock2(ControllerBindingWidget* parent)
	: ControllerBindingWidget_Base(parent)
{
	m_ui.setupUi(this);
	initBindingWidgets();
}

ControllerBindingWidget_DualShock2::~ControllerBindingWidget_DualShock2()
{
}

QIcon ControllerBindingWidget_DualShock2::getIcon() const
{
	return QIcon::fromTheme("controller-line");
}

ControllerBindingWidget_Base* ControllerBindingWidget_DualShock2::createInstance(ControllerBindingWidget* parent)
{
	return new ControllerBindingWidget_DualShock2(parent);
}

ControllerBindingWidget_Guitar::ControllerBindingWidget_Guitar(ControllerBindingWidget* parent)
	: ControllerBindingWidget_Base(parent)
{
	m_ui.setupUi(this);
	initBindingWidgets();
}

ControllerBindingWidget_Guitar::~ControllerBindingWidget_Guitar()
{
}

QIcon ControllerBindingWidget_Guitar::getIcon() const
{
	return QIcon::fromTheme("guitar-line");
}

ControllerBindingWidget_Base* ControllerBindingWidget_Guitar::createInstance(ControllerBindingWidget* parent)
{
	return new ControllerBindingWidget_Guitar(parent);
}

ControllerBindingWidget_Jogcon::ControllerBindingWidget_Jogcon(ControllerBindingWidget* parent)
	: ControllerBindingWidget_Base(parent)
{
	m_ui.setupUi(this);
	initBindingWidgets();
}

ControllerBindingWidget_Jogcon::~ControllerBindingWidget_Jogcon()
{
}

QIcon ControllerBindingWidget_Jogcon::getIcon() const
{
	return QIcon::fromTheme("jogcon-line");
}

ControllerBindingWidget_Base* ControllerBindingWidget_Jogcon::createInstance(ControllerBindingWidget* parent)
{
	return new ControllerBindingWidget_Jogcon(parent);
}

ControllerBindingWidget_Negcon::ControllerBindingWidget_Negcon(ControllerBindingWidget* parent)
	: ControllerBindingWidget_Base(parent)
{
	m_ui.setupUi(this);
	initBindingWidgets();
}

ControllerBindingWidget_Negcon::~ControllerBindingWidget_Negcon()
{
}

QIcon ControllerBindingWidget_Negcon::getIcon() const
{
	return QIcon::fromTheme("negcon-line");
}

ControllerBindingWidget_Base* ControllerBindingWidget_Negcon::createInstance(ControllerBindingWidget* parent)
{
	return new ControllerBindingWidget_Negcon(parent);
}

ControllerBindingWidget_Popn::ControllerBindingWidget_Popn(ControllerBindingWidget* parent)
	: ControllerBindingWidget_Base(parent)
{
	m_ui.setupUi(this);
	initBindingWidgets();
}

ControllerBindingWidget_Popn::~ControllerBindingWidget_Popn()
{
}

QIcon ControllerBindingWidget_Popn::getIcon() const
{
	return QIcon::fromTheme("Popn-line");
}

ControllerBindingWidget_Base* ControllerBindingWidget_Popn::createInstance(ControllerBindingWidget* parent)
{
	return new ControllerBindingWidget_Popn(parent);
}

//////////////////////////////////////////////////////////////////////////

USBDeviceWidget::USBDeviceWidget(QWidget* parent, ControllerSettingsWindow* dialog, u32 port)
	: QWidget(parent)
	, m_dialog(dialog)
	, m_config_section(fmt::format("USB{}", port + 1))
	, m_port_number(port)
{
	m_ui.setupUi(this);
	m_ui.groupBox->setTitle(tr("USB Port %1").arg(port + 1));

	populateDeviceTypes();
	populatePages();

	ControllerSettingWidgetBinder::BindWidgetToInputProfileString(
		m_dialog->getProfileSettingsInterface(), m_ui.deviceType, m_config_section, "Type", "None");

	connect(m_ui.deviceType, &QComboBox::currentIndexChanged, this, &USBDeviceWidget::onTypeChanged);
	connect(m_ui.deviceSubtype, &QComboBox::currentIndexChanged, this, &USBDeviceWidget::onSubTypeChanged);
	connect(m_ui.bindings, &QPushButton::clicked, this, &USBDeviceWidget::onBindingsClicked);
	connect(m_ui.settings, &QPushButton::clicked, this, &USBDeviceWidget::onSettingsClicked);
	connect(m_ui.automaticBinding, &QPushButton::clicked, this, &USBDeviceWidget::onAutomaticBindingClicked);
	connect(m_ui.clearBindings, &QPushButton::clicked, this, &USBDeviceWidget::onClearBindingsClicked);
}

USBDeviceWidget::~USBDeviceWidget() = default;

QIcon USBDeviceWidget::getIcon() const
{
	static constexpr const char* icons[][2] = {
		{"Pad", "wheel-line"}, // Wheel Device
		{"Msd", "msd-line"}, // Mass Storage Device
		{"singstar", "singstar-line"}, // Singstar
		{"logitech_usbmic", "mic-line"}, // Logitech USB Mic
		{"headset", "headset-line"}, // Logitech Headset Mic
		{"hidkbd", "keyboard-2-line"}, // HID Keyboard
		{"hidmouse", "mouse-line"}, // HID Mouse
		{"RBDrumKit", "drum-line"}, // Rock Band Drum Kit
		{"BuzzDevice", "buzz-controller-line"}, // Buzz Controller
		{"TranceVibrator", "trance-vibrator-line"}, // Trance Vibrator
		{"webcam", "eyetoy-line"}, // EyeToy
		{"beatmania", "keyboard-2-line"}, // BeatMania Da Da Da!! (Konami Keyboard)
		{"seamic", "seamic-line"}, // SEGA Seamic
		{"printer", "printer-line"}, // Printer
		{"Keyboardmania", "keyboardmania-line"}, // KeyboardMania
		{"guncon2", "guncon2-line"}, // GunCon 2
		{"DJTurntable", "dj-hero-line"}, // DJ Hero TurnTable
		{"Gametrak", "gametrak-line"}, // Gametrak Device
		{"RealPlay", "realplay-sphere-line"}, // RealPlay Device
		{"TrainController", "train-line"} // Train Controller
	};

	for (size_t i = 0; i < std::size(icons); i++)
	{
		if (m_device_type == icons[i][0])
			return QIcon::fromTheme(icons[i][1]);
	}

	return QIcon::fromTheme("usb-fill");
}

// Hide arcade USB devices (light gun/wheel/drums) from the USB tab
static bool isHiddenUsbDeviceType(const std::string_view type)
{
	return type == "guncon2" || type == "Pad" || type == "RBDrumKit";
}

void USBDeviceWidget::populateDeviceTypes()
{
	for (const auto& [name, display_name] : USB::GetDeviceTypes())
	{
		if (isHiddenUsbDeviceType(name))
			continue;
		m_ui.deviceType->addItem(qApp->translate("USB", display_name), QString::fromUtf8(name));
	}
}

void USBDeviceWidget::populatePages()
{
	m_device_type = m_dialog->getStringValue(m_config_section.c_str(), "Type", "None");
	m_device_subtype = m_dialog->getIntValue(m_config_section.c_str(), fmt::format("{}_subtype", m_device_type).c_str(), 0);

	{
		QSignalBlocker sb(m_ui.deviceSubtype);
		m_ui.deviceSubtype->clear();
		for (const char* subtype : USB::GetDeviceSubtypes(m_device_type))
			m_ui.deviceSubtype->addItem(qApp->translate("USB", subtype));
		m_ui.deviceSubtype->setCurrentIndex(m_device_subtype);
		m_ui.deviceSubtype->setVisible(m_ui.deviceSubtype->count() > 0);
	}

	if (m_bindings_widget)
	{
		m_ui.stackedWidget->removeWidget(m_bindings_widget);
		delete m_bindings_widget;
		m_bindings_widget = nullptr;
	}
	if (m_settings_widget)
	{
		m_ui.stackedWidget->removeWidget(m_settings_widget);
		delete m_settings_widget;
		m_settings_widget = nullptr;
	}

	if (isHiddenUsbDeviceType(m_device_type))
	{
		m_ui.deviceSubtype->setVisible(false);
		m_ui.bindings->setEnabled(false);
		m_ui.settings->setEnabled(false);
		updateHeaderToolButtons();
		return;
	}

	const std::span<const InputBindingInfo> bindings(USB::GetDeviceBindings(m_device_type, m_device_subtype));
	const std::span<const SettingInfo> settings(USB::GetDeviceSettings(m_device_type, m_device_subtype));
	m_ui.bindings->setEnabled(!bindings.empty());
	m_ui.settings->setEnabled(!settings.empty());

	if (!bindings.empty())
	{
		m_bindings_widget = USBBindingWidget::createInstance(m_device_type, m_device_subtype, bindings, this);
		if (m_bindings_widget)
		{
			m_ui.stackedWidget->addWidget(m_bindings_widget);
			m_ui.stackedWidget->setCurrentWidget(m_bindings_widget);
		}
	}

	if (!settings.empty())
	{
		m_settings_widget = new ControllerCustomSettingsWidget(
			settings, m_config_section, m_device_type + "_", "USB", m_dialog, m_ui.stackedWidget);
		m_ui.stackedWidget->addWidget(m_settings_widget);
	}

	updateHeaderToolButtons();
}

void USBDeviceWidget::onTypeChanged()
{
	populatePages();
	m_dialog->updateListDescription(m_port_number, this);
}

void USBDeviceWidget::onSubTypeChanged(int new_index)
{
	m_dialog->setIntValue(m_config_section.c_str(), fmt::format("{}_subtype", m_device_type).c_str(), new_index);
	onTypeChanged();
}

void USBDeviceWidget::updateHeaderToolButtons()
{
	const QWidget* current_widget = m_ui.stackedWidget->currentWidget();
	const QSignalBlocker bindings_sb(m_ui.bindings);
	const QSignalBlocker settings_sb(m_ui.settings);

	const bool is_bindings = (m_bindings_widget && current_widget == m_bindings_widget);
	const bool is_settings = (m_settings_widget && current_widget == m_settings_widget);
	m_ui.bindings->setChecked(is_bindings);
	m_ui.automaticBinding->setEnabled(is_bindings);
	m_ui.clearBindings->setEnabled(is_bindings);
	m_ui.settings->setChecked(is_settings);
}

void USBDeviceWidget::onBindingsClicked()
{
	if (!m_bindings_widget)
		return;

	m_ui.stackedWidget->setCurrentWidget(m_bindings_widget);
	updateHeaderToolButtons();
}

void USBDeviceWidget::onSettingsClicked()
{
	if (!m_settings_widget)
		return;

	m_ui.stackedWidget->setCurrentWidget(m_settings_widget);
	updateHeaderToolButtons();
}

void USBDeviceWidget::onAutomaticBindingClicked()
{
	QMenu menu(this);
	bool added = false;

	for (const QPair<QString, QString>& dev : m_dialog->getDeviceList())
	{
		// we set it as data, because the device list could get invalidated while the menu is up
		QAction* action;
		if (dev.first.compare(dev.second, Qt::CaseInsensitive) == 0)
			action = menu.addAction(dev.first);
		else
			action = menu.addAction(QStringLiteral("%1: %2").arg(dev.first).arg(dev.second));
		action->setData(dev.first);
		connect(action, &QAction::triggered, this, [this, action]() { doDeviceAutomaticBinding(action->data().toString()); });
		added = true;
	}

	if (!added)
	{
		QAction* action = menu.addAction(tr("No devices available"));
		action->setEnabled(false);
	}

	menu.exec(QCursor::pos());
}

void USBDeviceWidget::onClearBindingsClicked()
{
	if (QMessageBox::question(QtUtils::GetRootWidget(this), tr("Clear Bindings"),
			tr("Are you sure you want to clear all bindings for this device? This action cannot be undone.")) != QMessageBox::Yes)
	{
		return;
	}

	if (m_dialog->isEditingGlobalSettings())
	{
		{
			auto lock = Host::GetSettingsLock();
			USB::ClearPortBindings(*Host::Internal::GetBaseSettingsLayer(), m_port_number);
		}
		Host::CommitBaseSettingChanges();
	}
	else
	{
		USB::ClearPortBindings(*m_dialog->getProfileSettingsInterface(), m_port_number);
		m_dialog->getProfileSettingsInterface()->Save();
	}

	// force a refresh after clearing
	g_emu_thread->applySettings();
	onTypeChanged();
}

void USBDeviceWidget::doDeviceAutomaticBinding(const QString& device)
{
	std::vector<std::pair<GenericInputBinding, std::string>> mapping = InputManager::GetGenericBindingMapping(device.toStdString());
	if (mapping.empty())
	{
		QMessageBox::critical(QtUtils::GetRootWidget(this), tr("Automatic Binding"),
			tr("No generic bindings were generated for device '%1'. The controller/source may not support automatic mapping.").arg(device));
		return;
	}

	bool result;
	if (m_dialog->isEditingGlobalSettings())
	{
		{
			auto lock = Host::GetSettingsLock();
			result = USB::MapDevice(*Host::Internal::GetBaseSettingsLayer(), m_port_number, mapping);
		}
		if (result)
			Host::CommitBaseSettingChanges();
	}
	else
	{
		result = USB::MapDevice(*m_dialog->getProfileSettingsInterface(), m_port_number, mapping);
		if (result)
		{
			m_dialog->getProfileSettingsInterface()->Save();
			g_emu_thread->reloadInputBindings();
		}
	}

	// force a refresh after mapping
	if (result)
	{
		g_emu_thread->applySettings();
		onTypeChanged();
	}
}

//////////////////////////////////////////////////////////////////////////

USBBindingWidget::USBBindingWidget(USBDeviceWidget* parent)
	: QWidget(parent)
{
}

USBBindingWidget::~USBBindingWidget()
{
}

QIcon USBBindingWidget::getIcon() const
{
	return QIcon::fromTheme("controller-strike-line");
}

std::string USBBindingWidget::getBindingKey(const char* binding_name) const
{
	return USB::GetConfigSubKey(getDeviceType(), binding_name);
}

void USBBindingWidget::createWidgets(std::span<const InputBindingInfo> bindings)
{
	QGroupBox* axis_gbox = nullptr;
	QGridLayout* axis_layout = nullptr;
	QGroupBox* button_gbox = nullptr;
	QGridLayout* button_layout = nullptr;
	SettingsInterface* sif = getDialog()->getProfileSettingsInterface();

	QScrollArea* scrollarea = new QScrollArea(this);
	QWidget* scrollarea_widget = new QWidget(scrollarea);
	scrollarea->setWidget(scrollarea_widget);
	scrollarea->setWidgetResizable(true);
	scrollarea->setFrameShape(QFrame::StyledPanel);
	scrollarea->setFrameShadow(QFrame::Plain);

	// We do axes and buttons separately, so we can figure out how many columns to use.
	constexpr int NUM_AXIS_COLUMNS = 2;
	int column = 0;
	int row = 0;
	for (const InputBindingInfo& bi : bindings)
	{
		if (bi.bind_type == InputBindingInfo::Type::Axis || bi.bind_type == InputBindingInfo::Type::HalfAxis ||
			bi.bind_type == InputBindingInfo::Type::Pointer || bi.bind_type == InputBindingInfo::Type::Device)
		{
			if (!axis_gbox)
			{
				axis_gbox = new QGroupBox(tr("Axes"), scrollarea_widget);
				axis_layout = new QGridLayout(axis_gbox);
			}

			QGroupBox* gbox = new QGroupBox(qApp->translate("USB", bi.display_name), axis_gbox);
			QVBoxLayout* temp = new QVBoxLayout(gbox);
			InputBindingWidget* widget = new InputBindingWidget(gbox, sif, bi.bind_type, getConfigSection(), getBindingKey(bi.name));
			temp->addWidget(widget);
			axis_layout->addWidget(gbox, row, column);
			if ((++column) == NUM_AXIS_COLUMNS)
			{
				column = 0;
				row++;
			}
		}
	}
	if (axis_gbox)
		axis_layout->addItem(new QSpacerItem(1, 1, QSizePolicy::Minimum, QSizePolicy::Expanding), ++row, 0);

	const int num_button_columns = axis_layout ? 2 : 4;
	row = 0;
	column = 0;
	for (const InputBindingInfo& bi : bindings)
	{
		if (bi.bind_type == InputBindingInfo::Type::Button)
		{
			if (!button_gbox)
			{
				button_gbox = new QGroupBox(tr("Buttons"), scrollarea_widget);
				button_layout = new QGridLayout(button_gbox);
			}

			QGroupBox* gbox = new QGroupBox(qApp->translate("USB", bi.display_name), button_gbox);
			QVBoxLayout* temp = new QVBoxLayout(gbox);
			InputBindingWidget* widget = new InputBindingWidget(gbox, sif, bi.bind_type, getConfigSection(), getBindingKey(bi.name));
			temp->addWidget(widget);
			button_layout->addWidget(gbox, row, column);
			if ((++column) == num_button_columns)
			{
				column = 0;
				row++;
			}
		}
	}

	if (button_gbox)
		button_layout->addItem(new QSpacerItem(1, 1, QSizePolicy::Minimum, QSizePolicy::Expanding), ++row, 0);

	if (!axis_gbox && !button_gbox)
		return;

	QHBoxLayout* layout = new QHBoxLayout(scrollarea_widget);
	if (axis_gbox)
		layout->addWidget(axis_gbox);
	if (button_gbox)
		layout->addWidget(button_gbox);
	layout->addItem(new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Minimum));

	QHBoxLayout* main_layout = new QHBoxLayout(this);
	main_layout->addWidget(scrollarea);
}


void USBBindingWidget::bindWidgets(std::span<const InputBindingInfo> bindings)
{
	SettingsInterface* sif = getDialog()->getProfileSettingsInterface();

	for (const InputBindingInfo& bi : bindings)
	{
		if (bi.bind_type == InputBindingInfo::Type::Axis || bi.bind_type == InputBindingInfo::Type::HalfAxis ||
			bi.bind_type == InputBindingInfo::Type::Button || bi.bind_type == InputBindingInfo::Type::Pointer ||
			bi.bind_type == InputBindingInfo::Type::Device)
		{
			InputBindingWidget* widget = findChild<InputBindingWidget*>(QString::fromUtf8(bi.name));
			if (!widget)
			{
				Console.Error("(USBBindingWidget) No widget found for '%s'.", bi.name);
				continue;
			}

			widget->initialize(sif, bi.bind_type, getConfigSection(), getBindingKey(bi.name));
		}

		if (bi.bind_type == InputBindingInfo::Type::Motor)
		{
			InputVibrationBindingWidget* widget = findChild<InputVibrationBindingWidget*>(QString::fromUtf8(bi.name));
			if (widget)
				widget->setKey(getDialog(), getConfigSection(), getBindingKey(bi.name));
		}
	}
}

USBBindingWidget* USBBindingWidget::createInstance(
	const std::string& type, u32 subtype, std::span<const InputBindingInfo> bindings, USBDeviceWidget* parent)
{
	USBBindingWidget* widget = new USBBindingWidget(parent);
	bool has_template = false;

	if (type == "Pad")
	{
		if (subtype == 0) // Generic or Driving Force
		{
			Ui::USBBindingWidget_DrivingForce().setupUi(widget);
			has_template = true;
		}
		else if (subtype == 3) // GT Force
		{
			Ui::USBBindingWidget_GTForce().setupUi(widget);
			has_template = true;
		}
	}
	else if (type == "BuzzDevice")
	{
		Ui::USBBindingWidget_Buzz().setupUi(widget);
		has_template = true;
	}
	else if (type == "TrainController")
	{
		if (subtype == 0)
		{
			Ui::USBBindingWidget_DenshaCon().setupUi(widget);
			has_template = true;
		}
		else if (subtype == 1)
		{
			Ui::USBBindingWidget_ShinkansenCon().setupUi(widget);
			has_template = true;
		}
		else if (subtype == 2)
		{
			Ui::USBBindingWidget_RyojouhenCon().setupUi(widget);
			has_template = true;
		}
	}
	else if (type == "Gametrak")
	{
		Ui::USBBindingWidget_Gametrak().setupUi(widget);
		has_template = true;
	}
	else if (type == "guncon2")
	{
		Ui::USBBindingWidget_GunCon2().setupUi(widget);
		has_template = true;

		// Embed crosshair settings directly on the bindings page (no Settings subtab).
		QGridLayout* mainLayout = qobject_cast<QGridLayout*>(widget->layout());
		if (mainLayout)
		{
			QGroupBox* crosshairGroup = new QGroupBox(qApp->translate("USB", "Crosshair"), widget);
			QGridLayout* chLayout = new QGridLayout(crosshairGroup);
			chLayout->setColumnStretch(0, 0);
			chLayout->setColumnStretch(1, 1);
			SettingsInterface* sif = parent->getDialog()->getProfileSettingsInterface();
			const std::string& config_section = parent->getConfigSection();
			const std::string prefix = std::string(parent->getDeviceType()) + "_";

			// Cursor Path
			QLineEdit* cursorPath = new QLineEdit(crosshairGroup);
			cursorPath->setObjectName(QStringLiteral("cursor_path"));
			QPushButton* browseBtn = new QPushButton(qApp->translate("USB", "Browse..."), crosshairGroup);
			ControllerSettingWidgetBinder::BindWidgetToInputProfileString(
				sif, cursorPath, config_section, prefix + "cursor_path", "");
			QObject::connect(browseBtn, &QPushButton::clicked, [widget, cursorPath]() {
				const QString path(QDir::toNativeSeparators(QFileDialog::getOpenFileName(widget, qApp->translate("USB", "Select File"))));
				if (!path.isEmpty())
					cursorPath->setText(path);
			});
			QHBoxLayout* pathHBox = new QHBoxLayout();
			pathHBox->addWidget(cursorPath, 1);
			pathHBox->addWidget(browseBtn);
			chLayout->addWidget(new QLabel(qApp->translate("USB", "Cursor Path"), crosshairGroup), 0, 0);
			chLayout->addLayout(pathHBox, 0, 1);

			// Cursor Scale
			QDoubleSpinBox* cursorScale = new QDoubleSpinBox(crosshairGroup);
			cursorScale->setObjectName(QStringLiteral("cursor_scale"));
			cursorScale->setMinimum(1);
			cursorScale->setMaximum(1000);
			cursorScale->setSingleStep(1);
			cursorScale->setSuffix(QStringLiteral("%"));
			cursorScale->setDecimals(0);
			ControllerSettingWidgetBinder::BindWidgetToInputProfileFloat(
				sif, cursorScale, config_section, prefix + "cursor_scale", 1.0f, 100.0f);
			chLayout->addWidget(new QLabel(qApp->translate("USB", "Cursor Scale"), crosshairGroup), 1, 0);
			chLayout->addWidget(cursorScale, 1, 1);

			// Cursor Color
			QLineEdit* cursorColor = new QLineEdit(crosshairGroup);
			cursorColor->setObjectName(QStringLiteral("cursor_color"));
			ControllerSettingWidgetBinder::BindWidgetToInputProfileString(
				sif, cursorColor, config_section, prefix + "cursor_color", "#ffffff");
			chLayout->addWidget(new QLabel(qApp->translate("USB", "Cursor Color"), crosshairGroup), 2, 0);
			chLayout->addWidget(cursorColor, 2, 1);

			// Insert crosshair group before the vertical spacer (last item)
			int spacerRow = mainLayout->rowCount();
			mainLayout->addWidget(crosshairGroup, spacerRow, 0, 1, mainLayout->columnCount());

			// Sinden Lightgun Border — only on USB Port 1
			if (parent->getPortNumber() == 0)
			{
				QGroupBox* sindenGroup = new QGroupBox(qApp->translate("USB", "Light Gun Border (Sinden)"), widget);
				QGridLayout* sindenLayout = new QGridLayout(sindenGroup);
				sindenLayout->setColumnStretch(0, 0);
				sindenLayout->setColumnStretch(1, 1);

				QCheckBox* sindenEnabled = new QCheckBox(qApp->translate("USB", "Enable white border"), sindenGroup);
				sindenEnabled->setToolTip(qApp->translate("USB",
					"Display a white border around the screen for Sinden Lightgun tracking. Only active for lightgun games."));
				ControllerSettingWidgetBinder::BindWidgetToInputProfileBool(
					sif, sindenEnabled, ACJV::CONFIG_SECTION, "SindenBorderEnabled", false);
				sindenLayout->addWidget(sindenEnabled, 0, 0, 1, 2);

				QComboBox* sindenMode = new QComboBox(sindenGroup);
				sindenMode->addItem(qApp->translate("USB", "4:3 (Game Surface)"));
				sindenMode->addItem(qApp->translate("USB", "Fullscreen (Entire Window)"));
				sindenMode->setToolTip(qApp->translate("USB",
					"4:3 frames the rendered game surface. Fullscreen fills the entire emulator window."));
				ControllerSettingWidgetBinder::BindWidgetToInputProfileInt(
					sif, sindenMode, ACJV::CONFIG_SECTION, "SindenBorderMode", 0);
				sindenLayout->addWidget(new QLabel(qApp->translate("USB", "Border Mode"), sindenGroup), 1, 0);
				sindenLayout->addWidget(sindenMode, 1, 1);

				QSpinBox* sindenThickness = new QSpinBox(sindenGroup);
				sindenThickness->setMinimum(1);
				sindenThickness->setMaximum(100);
				sindenThickness->setValue(10);
				sindenThickness->setSuffix(QStringLiteral(" px"));
				sindenThickness->setToolTip(qApp->translate("USB", "Border thickness in pixels (1-100)"));
				ControllerSettingWidgetBinder::BindWidgetToInputProfileInt(
					sif, sindenThickness, ACJV::CONFIG_SECTION, "SindenBorderThickness", 10);
				sindenLayout->addWidget(new QLabel(qApp->translate("USB", "Border Thickness"), sindenGroup), 2, 0);
				sindenLayout->addWidget(sindenThickness, 2, 1);

				int sindenRow = mainLayout->rowCount();
				mainLayout->addWidget(sindenGroup, sindenRow, 0, 1, mainLayout->columnCount());
			}
		}
	}
	else if (type == "RealPlay")
	{
		Ui::USBBindingWidget_RealPlay().setupUi(widget);
		has_template = true;
	}
	else if (type == "TranceVibrator")
	{
		Ui::USBBindingWidget_TranceVibrator().setupUi(widget);
		has_template = true;
	}

	if (has_template)
		widget->bindWidgets(bindings);
	else
		widget->createWidgets(bindings);

	return widget;
}

#include "moc_ControllerBindingWidget.cpp"
