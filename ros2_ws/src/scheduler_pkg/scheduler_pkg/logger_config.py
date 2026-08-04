import logging
import sys

# 自定义日志格式，与您的输出风格一致
def setup_logger():
    logger = logging.getLogger("AuctionDemo")
    logger.setLevel(logging.INFO)

    # 输出到控制台
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.INFO)

    formatter = logging.Formatter(
        '%(asctime)s | %(levelname)s | %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)

    return logger

# 全局 logger 实例，供其他模块导入使用
logger = setup_logger()